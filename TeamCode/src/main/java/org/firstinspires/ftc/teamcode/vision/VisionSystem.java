// File: VisionSystem.java
package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.easyopencv.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.*;

public class VisionSystem {
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics (you need to calibrate your camera to get these values)
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // AprilTag size in meters
    double tagsize = 0.166;

    // Tag IDs
    int[] tagIds = {1, 2, 3}; // Example IDs

    private AprilTagDetection latestDetection = null;

    public VisionSystem(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Initialize the camera
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Create AprilTag pipeline
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT); // Adjust resolution and rotation
            }

            @Override
            public void onError(int errorCode) {
                // Handle error
            }
        });
    }

    public void update() {
        // Get latest detections
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection detection : detections) {
                // Check if the detection is one of our tags
                for (int id : tagIds) {
                    if (detection.id == id) {
                        latestDetection = detection;
                        break;
                    }
                }
            }
        } else {
            latestDetection = null;
        }
    }

    // Get the pose of the latest detected AprilTag
    public Pose2d getAprilTagPose() {
        if (latestDetection != null) {
            // Convert the detection pose to Pose2d
            double x = latestDetection.pose.x; // meters
            double y = latestDetection.pose.y; // meters
            double yaw = latestDetection.pose.yaw; // radians

            // Convert meters to the field coordinate system (adjust as needed)
            return new Pose2d(x, y, yaw);
        } else {
            return null;
        }
    }
}