package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/**
 * Standard tracking wheel localizer implementation.
 * Uses three dead wheels to track the robot's position on the field.
 */
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    // Constants: These need to be tuned for your robot
    public static double TICKS_PER_REV = 2000; // For an REV through-bore encoder
    public static double WHEEL_RADIUS = 0.6299; // In inches
    public static double GEAR_RATIO = 1.0; // Ratio between wheel and encoder

    public static double LATERAL_DISTANCE = 12.283; // Distance between the left and right wheels
    public static double FORWARD_OFFSET = 1.791; // Offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private final List<Integer> lastEncoderPositions;
    private final List<Integer> lastEncoderVelocities;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncoderPositions, List<Integer> lastTrackingEncoderVelocities) {
        super(Arrays.asList(
                // Positions of the tracking wheels relative to the robot center
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // Left wheel
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // Right wheel
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // Front wheel
        ));

        lastEncoderPositions = lastTrackingEncoderPositions;
        lastEncoderVelocities = lastTrackingEncoderVelocities;

        // Initialize the encoders using the hardware map
        leftEncoder = new Encoder(hardwareMap, "FrontL");
        rightEncoder = new Encoder(hardwareMap, "FrontR");
        frontEncoder = new Encoder(hardwareMap, "BackR");

        // Reverse any encoders if necessary
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.FORWARD);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncoderPositions.clear();
        lastEncoderPositions.add(leftPos);
        lastEncoderPositions.add(rightPos);
        lastEncoderPositions.add(frontPos);
        // Return the positions of the tracking wheels in inches
        return Arrays.asList(
                encoderTicksToInches(leftPos),
                encoderTicksToInches(rightPos),
                encoderTicksToInches(frontPos)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncoderVelocities.clear();
        lastEncoderVelocities.add(leftVel);
        lastEncoderVelocities.add(rightVel);
        lastEncoderVelocities.add(frontVel);
        // Return the velocities of the tracking wheels in inches per second
        return Arrays.asList(
                encoderTicksToInches(leftVel),
                encoderTicksToInches(rightVel),
                encoderTicksToInches(frontVel)
        );
    }
}