// File: Localization.java
package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.vision.VisionSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

/**
 * Handles localization by fusing odometry and vision data.
 */
public class Localization {
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private final VisionSystem visionSystem;
    private Pose2d currentPose;

    // Variables for odometry calculations
    private int lastLeftPosition = 0;
    private int lastRightPosition = 0;
    private int lastFrontPosition = 0;

    // Encoder ticks per revolution and wheel circumference
    private static final double TICKS_PER_REV = 8192; // Adjust based on your encoders
    private static final double WHEEL_RADIUS = 1; // in inches, adjust based on your wheels
    private static final double GEAR_RATIO = 1; // Adjust if you have gear reduction
    private static final double TRACK_WIDTH = 14; // Distance between left and right wheels

    public Localization(HardwareMap hardwareMap, VisionSystem visionSystem) {
        // Initialize encoders (assuming separate encoders for odometry wheels)
        leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
        frontEncoder = hardwareMap.get(DcMotor.class, "frontEncoder");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.visionSystem = visionSystem;
        currentPose = new Pose2d(0, 0, 0);
    }

    public void update() {
        // Update odometry readings
        Pose2d odometryPoseDelta = getOdometryDelta();

        // Update current pose with odometry delta
        currentPose = new Pose2d(
            currentPose.getX() + odometryPoseDelta.getX(),
            currentPose.getY() + odometryPoseDelta.getY(),
            currentPose.getHeading() + odometryPoseDelta.getHeading()
        );

        // Get vision-based pose if available
        Pose2d visionPose = visionSystem.getAprilTagPose();

        // Fuse odometry and vision data
        currentPose = fusePoses(currentPose, visionPose);
    }

    private Pose2d getOdometryDelta() {
        int leftPosition = leftEncoder.getCurrentPosition();
        int rightPosition = rightEncoder.getCurrentPosition();
        int frontPosition = frontEncoder.getCurrentPosition();

        int deltaLeft = leftPosition - lastLeftPosition;
        int deltaRight = rightPosition - lastRightPosition;
        int deltaFront = frontPosition - lastFrontPosition;

        lastLeftPosition = leftPosition;
        lastRightPosition = rightPosition;
        lastFrontPosition = frontPosition;

        // Convert encoder ticks to inches
        double deltaLeftInches = encoderTicksToInches(deltaLeft);
        double deltaRightInches = encoderTicksToInches(deltaRight);
        double deltaFrontInches = encoderTicksToInches(deltaFront);

        // Implement odometry calculations here (e.g., standard mecanum wheel odometry)
        double deltaX = (deltaLeftInches + deltaRightInches) / 2;
        double deltaY = deltaFrontInches;
        double deltaHeading = (deltaRightInches - deltaLeftInches) / TRACK_WIDTH;

        return new Pose2d(deltaX, deltaY, deltaHeading);
    }

    private double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private Pose2d fusePoses(Pose2d odometryPose, Pose2d visionPose) {
        // Implement sensor fusion algorithm
        if (visionPose != null) {
            // Simple fusion: weight vision more heavily
            double alpha = 0.8; // Weight for vision
            double beta = 1 - alpha; // Weight for odometry

            double fusedX = alpha * visionPose.getX() + beta * odometryPose.getX();
            double fusedY = alpha * visionPose.getY() + beta * odometryPose.getY();
            double fusedHeading = alpha * visionPose.getHeading() + beta * odometryPose.getHeading();

            return new Pose2d(fusedX, fusedY, fusedHeading);
        } else {
            return odometryPose;
        }
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }
}