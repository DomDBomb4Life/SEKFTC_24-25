// File: ThreeWheelOdometryLocalizer.java
package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ThreeWheelOdometryLocalizer {

    // Parameters for encoder positions (in tick units)
    public static class Params {
        public static double par0YTicks = -7.0; // Y position of the left encoder
        public static double par1YTicks = 7.0;  // Y position of the right encoder
        public static double perpXTicks = -5.0; // X position of the front encoder
    }

    // Encoders
    private final Encoder leftEncoder, rightEncoder, frontEncoder;

    // Conversion factor from encoder ticks to inches
    private final double inPerTick;

    // Previous encoder positions
    private int lastLeftPos, lastRightPos, lastFrontPos;
    private boolean initialized = false;

    public ThreeWheelOdometryLocalizer(HardwareMap hardwareMap) {
        // Initialize encoders using OverflowEncoder to handle potential overflow
        leftEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftEncoder")));
        rightEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightEncoder")));
        frontEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontEncoder")));

        // Set inPerTick based on your encoders
        double TICKS_PER_REV = 8192; // For REV Through Bore Encoders
        double WHEEL_RADIUS = 1;     // inches
        inPerTick = (2 * Math.PI * WHEEL_RADIUS) / TICKS_PER_REV;

        // Reverse encoders if necessary
        // leftEncoder.setDirection(Encoder.Direction.REVERSE);
        // rightEncoder.setDirection(Encoder.Direction.FORWARD);
        // frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public Twist2dDual<Time> update() {
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

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        // Calculate position deltas
        int leftPosDelta = leftPosVel.position - lastLeftPos;
        int rightPosDelta = rightPosVel.position - lastRightPos;
        int frontPosDelta = frontPosVel.position - lastFrontPos;

        // Calculate velocity deltas
        int leftVel = leftPosVel.velocity;
        int rightVel = rightPosVel.velocity;
        int frontVel = frontPosVel.velocity;

        // Convert ticks to inches
        double leftDistance = leftPosDelta * inPerTick;
        double rightDistance = rightPosDelta * inPerTick;
        double frontDistance = frontPosDelta * inPerTick;

        double leftVelocity = leftVel * inPerTick;
        double rightVelocity = rightVel * inPerTick;
        double frontVelocity = frontVel * inPerTick;

        // Compute heading change
        double headingChange = (rightDistance - leftDistance) / (Params.par1YTicks - Params.par0YTicks);
        double headingVelocity = (rightVelocity - leftVelocity) / (Params.par1YTicks - Params.par0YTicks);

        // Compute average lateral movement
        double lateralDistance = (leftDistance + rightDistance) / 2.0;
        double lateralVelocity = (leftVelocity + rightVelocity) / 2.0;

        // Adjust strafe movement for rotation
        double strafeDistance = frontDistance - headingChange * Params.perpXTicks;
        double strafeVelocity = frontVelocity - headingVelocity * Params.perpXTicks;

        // Build the twist
        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[]{lateralDistance, lateralVelocity}),
                        new DualNum<>(new double[]{strafeDistance, strafeVelocity})
                ),
                new DualNum<>(new double[]{headingChange, headingVelocity})
        );

        // Update last positions
        lastLeftPos = leftPosVel.position;
        lastRightPos = rightPosVel.position;
        lastFrontPos = frontPosVel.position;

        return twist;
    }
}