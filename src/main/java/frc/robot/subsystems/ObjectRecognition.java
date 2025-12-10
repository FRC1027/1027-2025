package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;

/**
 * The ObjectRecognition subsystem uses the Limelight camera to identify game pieces.
 * 
 * It uses a "Neural Network" (a type of AI) to look at the camera feed and say "Coral was detected" or "Algae was detected".
 */
public class ObjectRecognition extends SubsystemBase{

    public ObjectRecognition(){

    }

    /**
     * Reads the latest results from the Limelight camera.
     * 
     * It checks if the AI detected anything. If it did, it prints the name of the object (Coral or Algae) to the console.
     */
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
            .andThen(Commands.waitUntil(() -> 
                LimelightHelpers.getCurrentPipelineIndex("limelight") == 1
            ))
            .andThen(new InstantCommand(() -> readDetections()))
            .andThen(new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 0)))
            .andThen(Commands.waitUntil(() -> 
                LimelightHelpers.getCurrentPipelineIndex("limelight") == 0
            ));
    }
}