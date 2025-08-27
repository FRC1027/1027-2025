package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;

// Uncomment later if using WPILib's official AprilTag layout
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose3d;

/**
 * Autonomous routine:
 * 1. Drive forward a little.
 * 2. Find AprilTag ID 4 with Limelight and drive toward it.
 * 3. Stop ~0.3m (1 foot) in front of the tag (blocking).
 * 4. Align turret so Limelight centers on ID 4 (blocking).
 * 5. Fire shooter using ShooterSubsystem.TimedOuttake().
 *
 * Field-layout code is commented inline where steps 2–4 are.
 */
public class AutoShootAtTag4 extends SequentialCommandGroup {

    public AutoShootAtTag4(SwerveSubsystem drivebase, TurretSubsystem turret, ShooterSubsystem shooter) {
        addCommands(

            // Step 1: Move forward briefly (~1 foot)
            Commands.run(() -> drivebase.drive(
                        new Translation2d(0.25, 0.0), // forward 0.25 m/s
                        0.0,                           // no rotation
                        true                           // field-relative
                    ), drivebase)
                    .withTimeout(Units.feetToMeters(1) / 0.25)
                    .andThen(() -> drivebase.drive(
                        new Translation2d(0.0, 0.0),
                        0.0,
                        true
                    )), // stop

            // Step 2: Drive toward AprilTag ID 4 until ~0.3m away
            Commands.run(() -> {
                if (LimelightHelpers.getFiducialID("limelight") == 4) {
                    System.out.println("tracking id 4");
                    double distance = LimelightHelpers.getTargetPose3d_CameraSpace("limelight").getX();
                    if (distance > 0) {
                        System.out.println(distance);
                        drivebase.drive(new Translation2d(0.25, 0.0), 0.0, true);
                    } else {
                        System.out.println(distance);
                        drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true);
                    }
                } else {
                    System.out.println("id not found");
                    drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true);
                }

                /* --- OPTIONAL FIELD LAYOUT VERSION ---
                 * Uncomment this block and comment out the above Limelight chase code
                 * if using WPILib official field layout instead of paper tag.
                 *
                 * AprilTagFieldLayout fieldLayout = AprilTagFields.k2025Crescendo.loadAprilTagLayoutField();
                 * Pose3d tagPose = fieldLayout.getTagPose(4).get();  // ID 4 pose on field
                 * drivebase.driveToPose(tagPose.toPose2d());
                 * turret.trackTargetWithLimelight(); // align turret once at tag
                 */
            }, drivebase).withTimeout(3.0),

            // Step 2b: Block until distance ≤ 0.3m or 3s timeout
            new WaitUntilCommand(() ->
                LimelightHelpers.getFiducialID("limelight") == 4 &&
                LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ() <= 0.3
            ).withTimeout(3.0),

            // Step 3: Stop drivebase fully
            Commands.runOnce(() -> drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true), drivebase),

            // Step 4a: Actively adjust turret using Limelight
            Commands.run(() -> {
                if (LimelightHelpers.getFiducialID("limelight") == 4) {
                    turret.trackTargetWithLimelight();
                }
            }, turret).withTimeout(2.0),

            // Step 4b: Block until turret centered (|tx| < 1°) or 2s timeout
            new WaitUntilCommand(() ->
                LimelightHelpers.getFiducialID("limelight") == 4 &&
                Math.abs(LimelightHelpers.getTX("limelight")) < 1.0
            ).withTimeout(2.0),

            // Small pause to stabilize aim
            new WaitCommand(0.3),

            // Step 5: Shoot at AprilTag 4
            shooter.TimedOuttake()

            // --- OPTIONAL FAILSAFE ---
            // If AprilTag 4 not found within X seconds, skip to shooting anyway:
            // .deadlineWith(new WaitCommand(3.0).andThen(shooter.TimedOuttake()))
        );
    }
}
