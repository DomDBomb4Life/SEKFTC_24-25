//// File: VisionSystem.java
//package org.firstinspires.ftc.teamcode.vision;
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.*;
//import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.*;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.openftc.apriltag.AprilTagDetection;
//
//import com.acmerobotics.roadrunner.Pose2d;
//
//import java.util.List;
//
///**
// * Handles vision processing using the camera and AprilTag detection.
// */
//public class VisionSystem {
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTagProcessor;
//    private HardwareMap hardwareMap;
//
//    // Lens intrinsics (you may need to calibrate your camera to get these values)
//    private final double fx = 578.272;
//    private final double fy = 578.272;
//    private final double cx = 402.145;
//    private final double cy = 221.506;
//
//    // AprilTag size in meters
//    private final double tagsize = 0.166;
//
//    // Tag IDs of interest
//    private final int[] tagIds = {1, 2, 3}; // Example IDs
//
//    private AprilTagDetection latestDetection = null;
//
//    public VisionSystem(HardwareMap hardwareMap) {
//        this.hardwareMap = hardwareMap;
//
//        // Initialize the AprilTag processor
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCube(true)
//                .setDrawTagID(true)
//                .setTagsize(tagsize)
//                .setLensIntrinsics(fx, fy, cx, cy)
//                .build();
//
//        // Optionally set the IDs of interest
//        for (int id : tagIds) {
//            aprilTagProcessor.addAprilTagIdToTrack(id);
//        }
//
//        // Initialize the VisionPortal with the camera and processor
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTagProcessor)
//                .setCameraResolution(new Size(800, 448))
//                .enableCameraMonitoring(true)
//                .build();
//    }
//
//    public void update() {
//        // Get the latest detections
//        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
//
//        if (detections != null && !detections.isEmpty()) {
//            for (AprilTagDetection detection : detections) {
//                // Check if the detection is one of our tags
//                for (int id : tagIds) {
//                    if (detection.getMetadata().tagId == id) {
//                        latestDetection = detection;
//                        break;
//                    }
//                }
//            }
//        } else {
//            latestDetection = null;
//        }
//    }
//
//    // Get the pose of the latest detected AprilTag
//    public Pose2d getAprilTagPose() {
//        if (latestDetection != null) {
//            // Get the pose estimation from the detection
//            OpenGLMatrix pose = latestDetection.getPose();
//
//            if (pose != null) {
//                // Extract the x, y, and yaw from the pose
//                float[] translation = pose.getTranslation().getData();
//                double x = translation[0] / 1000.0; // Convert mm to meters
//                double y = translation[1] / 1000.0; // Convert mm to meters
//
//                // Extract yaw from the orientation matrix
//                Orientation orientation = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
//                double yaw = orientation.thirdAngle;
//
//                // Convert meters to field units (e.g., inches)
//                double xInches = x * 39.37;
//                double yInches = y * 39.37;
//
//                return new Pose2d(xInches, yInches, yaw);
//            }
//        }
//        return null;
//    }
//
//    // Additional method to get the ID of the latest detected tag
//    public int getLatestTagID() {
//        if (latestDetection != null) {
//            return latestDetection.getMetadata().tagId;
//        } else {
//            return -1; // Indicates no tag detected
//        }
//    }
//
//    // Call this method when the OpMode is stopped
//    public void close() {
//        if (visionPortal != null) {
//            visionPortal.close();
//            visionPortal = null;
//        }
//    }
//}