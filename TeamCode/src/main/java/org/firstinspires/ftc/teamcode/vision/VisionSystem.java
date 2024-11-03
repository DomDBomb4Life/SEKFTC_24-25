// File: VisionSystem.java
package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.concurrent.TimeUnit;

import java.util.List;

public class VisionSystem {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Constructor
    public VisionSystem(HardwareMap hardwareMap) {
        // Initialize the camera
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create the AprilTagProcessor using the pipeline class
        aprilTagProcessor = AprilTagDetectionPipeline.createAprilTagProcessor();

        // Build the VisionPortal
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    // Method to get the latest detections
    public List<AprilTagDetection> getLatestDetections() {
        return aprilTagProcessor.getDetections();
    }

    // Method to close the vision portal
    public void close() {
        visionPortal.close();
    }
    // Method to set manual exposure and gain
    public void setManualExposure(int exposureMs, int gain) {
        // Ensure the camera is streaming
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
        }
    }
}