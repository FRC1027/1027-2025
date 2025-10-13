package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveTowardTarget {
    
    private final SwerveSubsystem drivebase;
    private final double CAM_TO_BUMPER = 0.33; // meters (measure this)
    private final double STOP_DISTANCE = 1.5;  // meters from bumper to tag
    private final double MAX_SPEED = 0.4;      // forward speed (m/s)
    private final double MAX_ROTATION = 0.4;   // rotation speed (rad/s)
    private final int TARGET_ID = 4;           // tag to follow

    public DriveTowardTarget(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
    }

    public void execute() {
        if (LimelightHelpers.getFiducialID("limelight") != TARGET_ID) {
            SmartDashboard.putString("LL Status", "Tag ID not found");
            drivebase.drive(new Translation2d(0, 0), 0, true);
            return;
        }

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

        // Check if Limelight has a valid target
        double tv = limelight.getEntry("tv").getDouble(0.0);
        if (tv < 1.0) {
            SmartDashboard.putString("LL Status", "No target");
            drivebase.drive(new Translation2d(0, 0), 0, true);
            return;
        }

        double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
        if (pose == null || pose.length < 3) {
            SmartDashboard.putString("LL Status", "No pose data");
            drivebase.drive(new Translation2d(0, 0), 0, true);
            return;
        }

        double tx = pose[0]; // left/right offset (m)
        double ty = pose[1]; // vertical offset (m)
        double tz = pose[2]; // forward distance (m)

        double cameraToTagDist = Math.sqrt(tx * tx + ty * ty + tz * tz);
        double bumperToTagDist = Math.max(0.0, cameraToTagDist - CAM_TO_BUMPER);

        SmartDashboard.putNumber("LL tx (m)", tx);
        SmartDashboard.putNumber("LL tz (m)", tz);
        SmartDashboard.putNumber("LL bumper->tag (m)", bumperToTagDist);

        // --- Control logic ---
        double forwardSpeed = 0.0;
        double rotationSpeed = 0.0;

        // Forward/backward control (simple proportional)
        if (bumperToTagDist > STOP_DISTANCE) {
            forwardSpeed = MAX_SPEED * Math.min(1.0, bumperToTagDist / 4.0); // scale by distance
        }

        // Turn to face the tag (simple proportional steering)
        double kP_turn = 1.0; // tuning constant — increase for snappier turning
        rotationSpeed = -kP_turn * tx; // turn left if tag is to the left
        rotationSpeed = Math.max(-MAX_ROTATION, Math.min(MAX_ROTATION, rotationSpeed)); // clamp

        SmartDashboard.putNumber("LL ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("LL RotationSpeed", rotationSpeed);

        // Drive toward tag
        drivebase.drive(new Translation2d(forwardSpeed, 0.0), rotationSpeed, true);
    }
}