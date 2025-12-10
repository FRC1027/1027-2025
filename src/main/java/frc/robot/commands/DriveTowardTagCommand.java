package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * A command that drives the robot toward an AprilTag or aligns with it.
 * 
 * This command uses the Limelight camera to detect an AprilTag and:
 * Rotates the robot to face the tag.
 * Optionally drives forward until a certain distance is reached.
 * 
 * It is designed to be simple enough for beginners to understand basic PID-like control loops.
 */
public class DriveTowardTagCommand extends Command {
    
    private final SwerveSubsystem drivebase;
    
    // Constants for configuration
    private final double CAM_TO_BUMPER = 0.33; // meters (distance from camera lens to front bumper)
    private final double STOP_DISTANCE = 0.5;  // meters (target distance from bumper to tag)
    private final int TARGET_ID = 4;           // The AprilTag ID we want to chase
    
    // Maximum speeds
    private final double maxSpeed;      // forward speed limit (m/s)
    private final double maxRotation;   // rotation speed limit (rad/s)

    // Internal state variables
    private double forwardSpeed = 0.0;
    private double rotationSpeed = 0.0;
    
    // Initialize with a high value so we don't stop immediately if we haven't seen the tag yet
    private double bumperToTagDist = 999.0;

    /**
     * Creates a new DriveTowardTagCommand.
     * 
     * @param drivebase   The swerve drive subsystem used to move the robot.
     * @param maxSpeed    The maximum forward speed in meters per second. Set to 0 for align-only.
     * @param maxRotation The maximum rotation speed in radians per second.
     */
    public DriveTowardTagCommand(SwerveSubsystem drivebase, double maxSpeed, double maxRotation) {
        this.drivebase = drivebase;
        this.maxSpeed = maxSpeed;
        this.maxRotation = maxRotation;

        // Require the drivebase so no other drive commands run at the same time
        addRequirements(drivebase);
    }

    /**
     * Default constructor that uses default speeds (2.0 m/s, 2.0 rad/s).
     * 
     * @param drivebase The swerve drive subsystem.
     */
    public DriveTowardTagCommand(SwerveSubsystem drivebase) {
        this(drivebase, 2.0, 2.0);
    }

    @Override
    public void initialize() {
        // Runs once when the command starts
        SmartDashboard.putString("LL Status", "Searching for tag...");
        System.out.println("[DriveTowardTag] Initialized with MaxSpeed: " + maxSpeed);
    }

    @Override
    public void execute() {
        // Runs repeatedly while the command is active (approx. every 20ms)

        // 1. Check if we are tracking the correct tag ID
        if (LimelightHelpers.getFiducialID("limelight") != TARGET_ID) {
            SmartDashboard.putString("LL Status", "Tag ID not found");
            stopRobot();
            return;
        }

        // 2. Get the "tv" (Target Valid) value from Limelight NetworkTables
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        double tv = limelight.getEntry("tv").getDouble(0.0);

        if (tv < 1.0) {
            SmartDashboard.putString("LL Status", "No target visible");
            stopRobot();
            return;
        }

        // 3. Get the 3D pose of the tag relative to the camera
        // Format: [x, y, z, roll, pitch, yaw]
        double[] pose = limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);

        if (pose == null || pose.length < 3) {
            SmartDashboard.putString("LL Status", "Invalid pose data");
            stopRobot();
            return;
        }

        // Extract coordinates
        double tx = pose[0]; // Horizontal offset (left/right) in meters
        double ty = pose[1]; // Vertical offset (up/down) in meters
        double tz = pose[2]; // Forward distance (depth) in meters

        // Calculate straight-line distance
        double cameraToTagDist = Math.sqrt(tx * tx + ty * ty + tz * tz);
        
        // Calculate distance from bumper to tag
        bumperToTagDist = Math.max(0.0, cameraToTagDist - CAM_TO_BUMPER);

        // Update SmartDashboard for debugging
        SmartDashboard.putNumber("LL tx (m)", tx);
        SmartDashboard.putNumber("LL tz (m)", tz);
        SmartDashboard.putNumber("LL bumper->tag (m)", bumperToTagDist);

        // --- CONTROL LOGIC ---

        // A. Forward Speed Control
        // Only drive forward if we have a maxSpeed > 0 and are far enough away
        if (maxSpeed > 0 && bumperToTagDist > STOP_DISTANCE) {
            // Proportional control: slow down as we get closer
            // We divide by 4.0 to create a gentle deceleration curve
            double speedFactor = Math.min(1.0, bumperToTagDist / 4.0);
            forwardSpeed = maxSpeed * speedFactor;
        } else {
            forwardSpeed = 0.0;
        }

        // B. Rotation Control
        // Turn to face the tag (minimize tx)
        double kP_turn = 1.0; // Proportional gain for turning
        rotationSpeed = -kP_turn * tx; // Negative because positive tx means tag is to the right, so we turn right (negative Z rotation)
        
        // Clamp rotation speed to our maximum allowed limit
        rotationSpeed = Math.max(-maxRotation, Math.min(maxRotation, rotationSpeed));

        SmartDashboard.putNumber("LL ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("LL RotationSpeed", rotationSpeed);

        // Apply the calculated speeds to the robot
        // Translation2d(x, y) -> x is forward, y is left
        drivebase.drive(new Translation2d(forwardSpeed, 0), rotationSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when the command finishes or is interrupted
        stopRobot();
        SmartDashboard.putString("LL Status", interrupted ? "Interrupted" : "Arrived at Target");
        System.out.println("[DriveTowardTag] Ended");
    }

    @Override
    public boolean isFinished() {
        // Check if we should stop automatically
        
        // 1. If we lost the tag, stop (safety)
        boolean tagLost = LimelightHelpers.getFiducialID("limelight") != TARGET_ID;
        
        // 2. If we are driving forward (maxSpeed > 0) and reached the target, stop
        boolean reachedTarget = (maxSpeed > 0) && (bumperToTagDist <= STOP_DISTANCE);
        
        // Note: If maxSpeed is 0 (align only), we never "finish" based on distance. 
        // The driver must release the button to stop aligning.
        
        return tagLost || reachedTarget;
    }

    /**
     * Helper method to stop the robot completely.
     */
    private void stopRobot() {
        drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true);
    }
}
