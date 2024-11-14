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
    public static double TICKS_PER_REV = 8192; // For an REV through-bore encoder
    public static double WHEEL_RADIUS = 1.0; // In inches
    public static double GEAR_RATIO = 1.0; // Ratio between wheel and encoder

    public static double LATERAL_DISTANCE = 14.0; // Distance between the left and right wheels
    public static double FORWARD_OFFSET = -7.0; // Offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                // Positions of the tracking wheels relative to the robot center
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // Left wheel
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // Right wheel
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // Front wheel
        ));

        // Initialize the encoders using the hardware map
        leftEncoder = new Encoder(hardwareMap, "FrontL");
        rightEncoder = new Encoder(hardwareMap, "FrontR");
        frontEncoder = new Encoder(hardwareMap, "FrontL");

        // Reverse any encoders if necessary
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.FORWARD);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        // Return the positions of the tracking wheels in inches
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // Return the velocities of the tracking wheels in inches per second
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}