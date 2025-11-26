package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;

public class ObjectRecognition extends SubsystemBase{

    public ObjectRecognition(){

    }

    public void readDetections() {

        LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        
        // Neural network detections
        if (results.targets_Detector.length > 0) {
            LimelightTarget_Detector detection = results.targets_Detector[0];
            String className = detection.className;
            //double confidence = detection.confidence;
            //double area = detection.ta;

            if (className.equals("coral")){
                System.out.println("Coral was detected");
            } else if (className.equals("algae")) {
                System.out.println("Algae was detected");
            }
        } else {
            System.out.println("No object detected");
        }
    }

    public SequentialCommandGroup recognizeObjectsCommand() {
        return new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 1))
            .andThen(new WaitCommand(0.1))
            .andThen(new InstantCommand(() -> readDetections()))
            .andThen(new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 0)))
            .andThen(new WaitCommand(0.1));        
    }


}