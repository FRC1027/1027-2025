package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        System.out.println(results);


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

    public Command recognizeObjectsCommand() {
        return new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 1))
            .andThen(Commands.waitUntil(() -> 
                LimelightHelpers.getCurrentPipelineIndex("limelight") == 1
            ))
            .andThen(Commands.runOnce(() -> readDetections()))
            .andThen(new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 0)));
    }
}