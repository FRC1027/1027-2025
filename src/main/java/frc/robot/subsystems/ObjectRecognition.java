package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;

public class ObjectRecognition extends SubsystemBase{

    public ObjectRecognition(){

    }

    public void recognizeObjects() {
        // Switch to pipeline 1
        LimelightHelpers.setPipelineIndex("limelight", 1);

        // wait for LL to actually process the new pipeline
        // Limelight runs at 22ms/frame meaning 50ms wait guarantees at least 2 frames processed.
        try { Thread.sleep(50); } catch (Exception e) {}

        LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        
        System.out.println(results); //frc.robot.LimelightHelpers$LimelightResults@1d1f8dd
        System.out.println(results.targets_Detector); //[Lfrc.robot.LimelightHelpers$LimelightTarget_Detector;@96b474
        System.out.println(results.targets_Detector.length); //0
        //System.out.println(results.targets_Detector[0].className); //crashes robot code if length is 0

        // Neural network detections
        if (results.targets_Detector.length > 0) {
            LimelightTarget_Detector detection = results.targets_Detector[0];
            String className = detection.className;
            //double confidence = detection.confidence;
            //double area = detection.ta;

            System.out.println(className);

            if (className.equals("coral")){
                System.out.println("Coral was detected");
            } else if (className.equals("algae")) {
                System.out.println("Algae was detected");
            }
        } else {
            System.out.println("No object detected");
        }
        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    public InstantCommand recognizeObjectsCommand() {
        return new InstantCommand(this::recognizeObjects, this);
    }
}