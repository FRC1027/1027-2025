package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.time.LocalTime;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;

// (Optional) Use WPILib’s official AprilTag field layout instead of Limelight pipeline
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose3d;


/**
 * Autonomous routine to locate AprilTag ID 4 and shoot:
 *
 * Sequence:
 *  1. Drive forward ~1 foot to get unstuck/closer to tag view.
 *  2. Find AprilTag ID 4 with Limelight until within ~1.5m of bumper.
 *  3. Stop and hold position.
 *  4. Align turret with tag using Limelight:
 *     - Actively adjust for 2s,
 *     - Then block until centered (|tx| < 1°) or 2s timeout.
 *  5. Fire shooter with ShooterSubsystem.TimedOuttake().
 *
 * Optional: Replace Limelight logic with official WPILib AprilTag field layout.
 */
public class AutoShootAtTag4 extends SequentialCommandGroup {

    public AutoShootAtTag4(SwerveSubsystem drivebase, TurretSubsystem turret, ShooterSubsystem shooter) {
        addCommands(

            // Step 1: Drive forward a short distance (~1 ft)
            Commands.run(() -> drivebase.drive(
                        new Translation2d(0.25, 0.0),    // forward velocity of 0.25 m/s
                        0.0,                        // no rotation
                        true                   // field-relative
                    ), drivebase)
                    .withTimeout(Units.feetToMeters(1)) // run long enough to cover ~1 ft
                    .andThen(() -> drivebase.drive(          // then stop
                        new Translation2d(0.0, 0.0),
                        0.0,
                        true
                    )),

            // Step 2: Drive toward AprilTag ID 4 using Limelight until ~1.5m from bumper
            Commands.run(() -> {

                
                // If statement that checks if Limelight ID 4 is being tracked
                if (LimelightHelpers.getFiducialID("limelight") == 4) {
                    System.out.println("tracking id 4");
                    
                    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
                    
                    // Read the "tv" (target valid) entry:
                    //   - tv = 1 → Limelight has a valid target
                    //   - tv = 0 → No valid target in view
                    double tv = limelight.getEntry("tv").getDouble(0.0);
                    if (tv < 1.0) {
                        SmartDashboard.putString("LL Status", "No target");
                        return;
                    }
                    
                    // Checks to see if the returned pose relative to camera pose [x,y,z,roll,pitch,yaw], were successfully
                    // returned. It also checks to see if the robot is less than 3 meters from the target.
                    double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
                    if (pose == null || pose.length < 3) {
                        SmartDashboard.putString("LL Status", "No pose array");
                        return;
                    }
                    
                    double tx = pose[0];   // horizontal offset (m)
                    double ty = pose[1];   // vertical offset (m)
                    double tz = pose[2];   // forward distance from CAMERA to tag (m)
                
                    // Computes straight-line distance from camera to tag (Euclidean)
                    double cameraToTagDist = Math.sqrt(tx*tx + ty*ty + tz*tz);
                
                    // Convert Camera -> Tag distance to BUMPER -> Tag distance by subtracting camToBumper offset
                    double camToBumper = 0.3302; // <--- measure this on your robot (meters)
                    double bumperToTagDist = Math.max(0.0, cameraToTagDist - camToBumper);
                    
                    // Display camera values on the Smartdashboard (for debugging purposes)
                    SmartDashboard.putNumber("LL tx (m)", tx);
                    SmartDashboard.putNumber("LL ty (m)", ty);
                    SmartDashboard.putNumber("LL tz (m)", tz);
                    SmartDashboard.putNumber("LL camera->tag (m)", cameraToTagDist);
                    SmartDashboard.putNumber("LL bumper->tag (m)", bumperToTagDist);

                    // stop threshold (1.5 m from bumper)
                    double stopDistance = 1.5;
                    
                    if (bumperToTagDist >= stopDistance) {
                        System.out.println(bumperToTagDist);
                        drivebase.drive(new Translation2d(0.25, 0.0), 0.0, true);
                    } else {
                        System.out.println(bumperToTagDist);
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
            }, drivebase).withTimeout(10.0),

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