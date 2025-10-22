package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveTowardTagCommand extends Command {
    
    private final SwerveSubsystem drivebase;
    private final double CAM_TO_BUMPER = 0.33; // meters (measure this)
    private final double STOP_DISTANCE = 0.5;  // meters from bumper to tag
    private final double MAX_SPEED = 3;      // forward speed (m/s)
    private final double MAX_ROTATION = 0.4;   // rotation speed (rad/s)
    private final int TARGET_ID = 4;           // tag to follow

    // --- Control logic ---
    private double forwardSpeed = 0.0;
    private double rotationSpeed = 0.0;

    // Having this varible, 'bumperToTagDist', set at a extremely high number prevents this command
    // from ending prematurely if it sees that the distance was already below the 'STOP_DISTANCE' constant.
    private double bumperToTagDist = 999.0;

    public DriveTowardTagCommand(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;

        // This prevents any other command that also requires drivebase from running simultaneously. If one 
        // is already running, the scheduler will interrupt it.
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        // Runs once when the command is first scheduled

        // Logs status to SmartDashboard
        SmartDashboard.putString("LL Status", "Searching for tag...");
        System.out.println("[DriveTowardTag] Initialized");
    }

    @Override
    public void execute() {
        // Called repeatedly (about every 20 ms) while the command is active

        /*
         * A series of if statements are performed to ensure that:
         * 
         * (1.) A valid AprilTag is being tracked,
         * (2.) the Limelight has detected a valid target (i.e., an AprilTag), 
         * (3.) and if the appropriate camera pose values (tx, ty, and tz) are being retrieved.
         * 
         * These if statements provide a "safety" check and are purposely redundant to prevent
         * targeting mistakes.
         */
        
        // If statement that checks if Limelight ID 4 is being tracked
        if (LimelightHelpers.getFiducialID("limelight") != TARGET_ID) {
            SmartDashboard.putString("LL Status", "Tag ID not found");
            drivebase.drive(new Translation2d(0, 0), 0, true);
            return;
        }

        // Access Limelight network table for live data
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

        // "tv" = 1 if valid target detected, 0 if not
        double tv = limelight.getEntry("tv").getDouble(0.0);

        // Checks if the Limelight has a valid target (i.e., an AprilTag)
        if (tv < 1.0) {
            SmartDashboard.putString("LL Status", "No target");
            drivebase.drive(new Translation2d(0, 0), 0, true);
            return;
        }

        // Pose array gives position of tag relative to the camera:
        // [x, y, z, roll, pitch, yaw]
        double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);

        // Checks to see if the returned camera pose data is valid
        if (pose == null || pose.length < 3) {
            SmartDashboard.putString("LL Status", "No pose data");
            drivebase.drive(new Translation2d(0, 0), 0, true);
            return;
        }

        // Extract x, y, z from the pose array
        double tx = pose[0]; // left/right offset (m)
        double ty = pose[1]; // vertical offset (m)
        double tz = pose[2]; // forward distance (m)

        // Computes straight-line distance from camera to tag (Euclidean)
        double cameraToTagDist = Math.sqrt(tx * tx + ty * ty + tz * tz);

        // Convert to bumper distance by subtracting camera offset
        bumperToTagDist = Math.max(0.0, cameraToTagDist - CAM_TO_BUMPER);

        SmartDashboard.putNumber("LL tx (m)", tx);
        SmartDashboard.putNumber("LL tz (m)", tz);
        SmartDashboard.putNumber("LL bumper->tag (m)", bumperToTagDist);

        // ---------- CONTROL LOGIC ----------

        // Forward speed control:
        //  - Move forward faster if we're farther from the tag
        //  - Stop once within STOP_DISTANCE
        if (bumperToTagDist > STOP_DISTANCE) {
            forwardSpeed = MAX_SPEED * Math.min(1.0, bumperToTagDist / 4.0); // scale by distance
        } else {
            forwardSpeed = 0.0;
        }

        // Turning control (simple proportional control)
        //  - Turn toward the tag if it's offset horizontally
        double kP_turn = 5.0; // tuning constant — increase for snappier turning
        rotationSpeed = -kP_turn * tx; // negative to rotate toward target
        rotationSpeed = Math.max(-MAX_ROTATION, Math.min(MAX_ROTATION, rotationSpeed)); // clamp to limits

        SmartDashboard.putNumber("LL ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("LL RotationSpeed", rotationSpeed);

        // Send movement command to drivetrain
        drivebase.drive(new Translation2d(forwardSpeed, 0.0), rotationSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs once when the command finishes or is interrupted by another command

        // Stops the robot
        drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true);

        // Logs status to SmartDashboard
        SmartDashboard.putString("LL Status", interrupted ? "Drive interrupted" : "Drive complete");
        System.out.println("[DriveTowardTag] Ended");
    }

    @Override
    public boolean isFinished() {
        // Called after each execute() to determine if the command should stop running

        // If the Limelight loses the tag or we've reached the stop distance, stop
        boolean tagLost = LimelightHelpers.getFiducialID("limelight") != TARGET_ID;
        return bumperToTagDist <= STOP_DISTANCE || tagLost;
    }
}