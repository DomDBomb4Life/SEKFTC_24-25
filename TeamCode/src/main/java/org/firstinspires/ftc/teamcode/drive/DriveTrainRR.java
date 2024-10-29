// File: DriveTrainRR.java
package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.List;

/**
 * DriveTrain class integrated with Roadrunner for autonomous path planning.
 */
public class DriveTrainRR extends MecanumDrive {

    // Motors
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Constants for the drive
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0);

    // Ticks per revolution and wheel radius
    public static final double TICKS_PER_REV = 537.6;
    public static final double WHEEL_RADIUS = 2; // in inches

    public DriveTrainRR(HardwareMap hardwareMap) {
        super(0, 0, 0, 0);

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontL");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontR");
        backRight = hardwareMap.get(DcMotorEx.class, "BackR");

        // Set motor directions
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set motors to use encoders
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the localizer to our custom localizer
        setLocalizer(new ThreeWheelOdometryLocalizer(hardwareMap));
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(frontLeft.getCurrentPosition()),
                encoderTicksToInches(backLeft.getCurrentPosition()),
                encoderTicksToInches(backRight.getCurrentPosition()),
                encoderTicksToInches(frontRight.getCurrentPosition())
        );
    }

    private double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        backLeft.setPower(v1);
        backRight.setPower(v2);
        frontRight.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        // Implement if you have a gyro sensor, otherwise return 0
        return 0;
    }
}