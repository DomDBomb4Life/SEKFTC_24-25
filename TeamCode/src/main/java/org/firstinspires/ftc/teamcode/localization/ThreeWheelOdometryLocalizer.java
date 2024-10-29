// File: ThreeWheelOdometryLocalizer.java
package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

public class ThreeWheelOdometryLocalizer extends TwoTrackingWheelLocalizer {

    // Constants
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // inches
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    // The positions of the tracking wheels relative to the robot's center
    public static Pose2d LEFT_WHEEL_POSE = new Pose2d(0, 7, 0);   // Left wheel
    public static Pose2d RIGHT_WHEEL_POSE = new Pose2d(0, -7, 0); // Right wheel
    public static Pose2d FRONT_WHEEL_POSE = new Pose2d(-7, 0, Math.toRadians(90)); // Front wheel

    private DcMotorEx leftEncoder, rightEncoder, frontEncoder;

    public ThreeWheelOdometryLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(LEFT_WHEEL_POSE, RIGHT_WHEEL_POSE));

        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "frontEncoder");

        leftEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition())
        );
    }

    public double getFrontWheelPosition() {
        return encoderTicksToInches(frontEncoder.getCurrentPosition());
    }

    private double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        // If you have a gyro sensor, return its heading here
        return 0;
    }
}