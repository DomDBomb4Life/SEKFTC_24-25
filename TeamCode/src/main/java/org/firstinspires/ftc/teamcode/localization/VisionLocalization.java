//// File: VisionLocalization.java
//package org.firstinspires.ftc.teamcode.localization;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.vision.VisionSystem;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//import java.util.List;
//
//public class VisionLocalization {
//
//    private VisionSystem visionSystem;
//
//    // Constructor
//    public VisionLocalization(VisionSystem visionSystem) {
//        this.visionSystem = visionSystem;
//    }
//
//    // Method to get the robot's pose based on AprilTag detections
//    public Pose2d getVisionPose() {
//        List<AprilTagDetection> detections = visionSystem.getLatestDetections();
//
//        if (detections == null || detections.isEmpty()) {
//            return null; // No detections
//        }
//
//        double totalX = 0;
//        double totalY = 0;
//        double totalHeading = 0;
//
//        int count = 0;
//
//        for (AprilTagDetection detection : detections) {
//            Pose3D robotPose3D = detection.robotPose;
//            if (robotPose3D != null) {
//                double poseX = robotPose3D.getPosition().x; // Get x from Position
//                double poseY = robotPose3D.getPosition().y; // Get y from Position
//                double poseHeading = robotPose3D.getOrientation().getYaw(); // Get yaw as heading in radians
//
//                totalX += poseX;
//                totalY += poseY;
//                totalHeading += poseHeading;
//
//                count++;
//            }
//        }
//
//        if (count == 0) {
//            return null; // No valid detections
//        }
//
//        // Compute average pose
//        double avgX = totalX / count;
//        double avgY = totalY / count;
//        double avgHeading = totalHeading / count;
//
//        // Convert from meters to inches
//        avgX *= 39.3701;
//        avgY *= 39.3701;
//
//        // Create and return Pose2d
//        return new Pose2d(avgX, avgY, avgHeading);
//    }
//}