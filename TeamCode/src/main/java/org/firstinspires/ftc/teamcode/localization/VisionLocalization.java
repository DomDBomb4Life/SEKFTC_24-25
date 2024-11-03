// File: VisionLocalization.java
package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.vision.VisionSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class VisionLocalization  {

    private VisionSystem visionSystem;

    // Previous pose estimates
    private Pose2d lastPose;
    private long lastTimestamp;
    private boolean initialized = false;

    // Constructor
    public VisionLocalization(VisionSystem visionSystem) {
        this.visionSystem = visionSystem;
    }

    // Method to update robot's pose based on AprilTag detections
    public Twist2dDual<Time> update() {
        List<AprilTagDetection> detections = visionSystem.getLatestDetections();

        long currentTimestamp = System.nanoTime();

        if (detections == null || detections.isEmpty()) {
            // No detections; return zero twist
            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        // Use multiple tag detections to estimate pose
        double x = 0;
        double y = 0;
        double heading = 0;

        int count = 0;

        for (AprilTagDetection detection : detections) {
            Pose3D robotPose3D = detection.robotPose;
            if (robotPose3D != null) {
                // Get the robot's pose from the detection
                double poseX = robotPose3D.getX(); // in meters
                double poseY = robotPose3D.getY();
                double poseHeading = robotPose3D.getHeading(); // in radians

                x += poseX;
                y += poseY;
                heading += poseHeading;

                count++;
            }
        }

        if (count == 0) {
            // No valid detections; return zero twist
            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        // Compute average pose
        x /= count;
        y /= count;
        heading /= count;

        // Convert from meters to inches
        x = x * 39.3701; // meters to inches
        y = y * 39.3701;

        Pose2d currentPose = new Pose2d(x, y, heading);

        if (!initialized) {
            // Initialize previous pose on first run
            lastPose = currentPose;
            lastTimestamp = currentTimestamp;
            initialized = true;

            // Return zero twist
            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        // Compute time delta
        double deltaTime = (currentTimestamp - lastTimestamp) / 1e9; // Convert nanoseconds to seconds

        // Compute position deltas
        double deltaX = currentPose.position().getX() - lastPose.position().getX();
        double deltaY = currentPose.position().getY() - lastPose.position().getY();
        double deltaHeading = currentPose.heading() - lastPose.heading();

        // Compute velocities
        double velocityX = deltaX / deltaTime;
        double velocityY = deltaY / deltaTime;
        double velocityHeading = deltaHeading / deltaTime;

        // Build the twist
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[]{deltaX, velocityX}),
                        new DualNum<>(new double[]{deltaY, velocityY})
                ),
                new DualNum<>(new double[]{deltaHeading, velocityHeading})
        );

        // Update last pose and timestamp
        lastPose = currentPose;
        lastTimestamp = currentTimestamp;

        return twist;
    }

    @Override
    public Pose2d getPoseEstimate() {
        List<AprilTagDetection> detections = visionSystem.getLatestDetections();

        if (detections == null || detections.isEmpty()) {
            return null; // No detections
        }

        // Use multiple tag detections to estimate pose
        double x = 0;
        double y = 0;
        double heading = 0;

        int count = 0;

        for (AprilTagDetection detection : detections) {
            Pose3D robotPose3D = detection.robotPose;
            if (robotPose3D != null) {
                // Get the robot's pose from the detection
                double poseX = robotPose3D.getX(); // in meters
                double poseY = robotPose3D.getY();
                double poseHeading = robotPose3D.getHeading(); // in radians

                x += poseX;
                y += poseY;
                heading += poseHeading;

                count++;
            }
        }

        if (count == 0) {
            return null;
        }

        // Compute average pose
        x /= count;
        y /= count;
        heading /= count;

        // Convert from meters to inches
        x = x * 39.3701; // meters to inches
        y = y * 39.3701;

        // Create and return Pose2d
        return new Pose2d(x, y, heading);
    }

    @Override
    public void setPoseEstimate(Pose2d pose2d) {
        // Optionally, you can set the lastPose to the provided pose
        this.lastPose = pose2d;
        this.initialized = true;
    }
}