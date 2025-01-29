package frc.robot.subsystems.subsystems2024.LimeLight;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LLDashBoard {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    public double getAprilTagX(){
        return x;
    }
    public double getAprilTagY(){
        return y;
    }
    public double getAprilTagArea(){
        return area;
    }
    //post to smart dashboard periodically
    /*
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    */
    public LLDashBoard(){
        //Shuffleboard.getTab("Drive Time").add("LimelightX", x);
        //Shuffleboard.getTab("Drive Time").add("Limelight Y", y);
        //Shuffleboard.getTab("Drive Time").add("LimelightArea", area);
        //Shuffleboard.getTab("Drive Time").add("Limelight Stream",limelightFeed);
        //CameraServer.startAutomaticCapture("limelightfeed","http://limelight.local:5800/stream.mjpg");
        
    }
}
