// File: ThreeWheelOdometryLocalizer.java
package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;

public class ThreeWheelOdometryLocalizer {

    // Parameters for encoder positions (in tick units)
    public static class Params {
        public static double par0YTicks = 0.0; // Y position of the left encoder
        public static double par1YTicks = 1.0; // Y position of the right encoder
        public static double perpXTicks = 0.0; // X position of the front encoder
    }

    // Encoders
    private final Encoder leftEncoder, rightEncoder, frontEncoder;

    // Conversion factor from encoder ticks to inches
    private final double inPerTick;

    // Previous encoder positions
    private int lastLeftPos, lastRightPos, lastFrontPos;
    private boolean initialized = false;

    // Current pose estimate
    private Pose2d poseEstimate;

    public ThreeWheelOdometryLocalizer(HardwareMap hardwareMap) {
        // Initialize encoders using OverflowEncoder to handle potential overflow
        leftEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftEncoder")));
        rightEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightEncoder")));
        frontEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontEncoder")));

        // Set inPerTick based on your encoders
        double TICKS_PER_REV = 8192;
        double WHEEL_RADIUS = 1; // inches
        inPerTick = (2 * Math.PI * WHEEL_RADIUS) / TICKS_PER_REV;

        // Initialize pose estimate at origin with zero rotation
        poseEstimate = new Pose2d(new Vector2d(0, 0), new Rotation2d(1, 0));
    }

    public void update() {
        // Get current positions and velocities from encoders
        PositionVelocityPair leftPosVel = leftEncoder.getPositionAndVelocity();
        PositionVelocityPair rightPosVel = rightEncoder.getPositionAndVelocity();
        PositionVelocityPair frontPosVel = frontEncoder.getPositionAndVelocity();

        if (!initialized) {
            // Initialize previous positions on the first run
            initialized = true;
            lastLeftPos = leftPosVel.position;
            lastRightPos = rightPosVel.position;
            lastFrontPos = frontPosVel.position;
            return;
        }

        // Calculate position deltas
        int leftPosDelta = leftPosVel.position - lastLeftPos;
        int rightPosDelta = rightPosVel.position - lastRightPos;
        int frontPosDelta = frontPosVel.position - lastFrontPos;

        // Convert ticks to inches
        double leftDistance = leftPosDelta * inPerTick;
        double rightDistance = rightPosDelta * inPerTick;
        double frontDistance = frontPosDelta * inPerTick;

        // Compute heading change
        double headingChange = (rightDistance - leftDistance) / (Params.par0YTicks - Params.par1YTicks);

        // Compute average lateral movement
        double lateralDistance = (leftDistance + rightDistance) / 2.0;

        // Adjust strafe movement for rotation
        double strafeDistance = frontDistance - headingChange * Params.perpXTicks;

        // Calculate global (field-centric) movement
        double theta = poseEstimate.heading.log();
        double dx = lateralDistance * Math.cos(theta) - strafeDistance * Math.sin(theta);
        double dy = lateralDistance * Math.sin(theta) + strafeDistance * Math.cos(theta);

        // Update pose estimate using Twist2d for consistency with Roadrunner geometry
        Twist2d twist = new Twist2d(new Vector2d(dx, dy), headingChange);
        poseEstimate = poseEstimate.plus(twist);

        // Update last positions
        lastLeftPos = leftPosVel.position;
        lastRightPos = rightPosVel.position;
        lastFrontPos = frontPosVel.position;
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void setPoseEstimate(Pose2d pose) {
        this.poseEstimate = pose;
    }
}