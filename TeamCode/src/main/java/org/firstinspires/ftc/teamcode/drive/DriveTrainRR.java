// File: DriveTrainRR.java
package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.localization.ThreeWheelOdometryLocalizer;

public class DriveTrainRR {

    // Motors
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Mecanum Kinematics
    private final MecanumKinematics kinematics;

    // Constants for the drive
    public static final double TRACK_WIDTH = 14; // inches
    public static final double WHEEL_RADIUS = 2; // inches
    public static final double GEAR_RATIO = 1;   // output (wheel) speed / input (motor) speed
    public static final double LATERAL_MULTIPLIER = 1.0; // Adjust if needed

    // Max RPM
    public static final double MAX_RPM = 340; // For NeveRest Orbital 20 motors

    // Feedforward parameters (to be tuned)
    public static final double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static final double kA = 0;
    public static final double kStatic = 0;

    // Ticks per revolution
    public static final double TICKS_PER_REV = 537.6; // For NeveRest Orbital 20 motors

    // Voltage sensor
    private final VoltageSensor voltageSensor;

    // Localizer
    public final ThreeWheelOdometryLocalizer localizer;
    public Pose2d poseEstimate;

    // IMU
    private final IMU imu;

    public DriveTrainRR(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontL");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackL");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontR");
        backRight = hardwareMap.get(DcMotorEx.class, "BackR");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize kinematics
        kinematics = new MecanumKinematics(TRACK_WIDTH, LATERAL_MULTIPLIER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize localizer
        localizer = new ThreeWheelOdometryLocalizer(hardwareMap);

        // Initialize pose estimate
        poseEstimate = new Pose2d(0, 0, 0);
    }

    public void setDrivePower(PoseVelocity2d drivePower) {
        // Convert desired velocities into wheel velocities
        MecanumKinematics.WheelVelocities<Time> wheelVelocities = kinematics.inverse(PoseVelocity2dDual.constant(drivePower, 1));

        // Normalize wheel velocities
        double maxPower = Math.max(
                Math.max(Math.abs(wheelVelocities.leftFront.get(0)), Math.abs(wheelVelocities.leftBack.get(0))),
                Math.max(Math.abs(wheelVelocities.rightFront.get(0)), Math.abs(wheelVelocities.rightBack.get(0)))
        );
        if (maxPower > 1.0) {
            wheelVelocities = new MecanumKinematics.WheelVelocities<>(
                    wheelVelocities.leftFront.div(maxPower),
                    wheelVelocities.leftBack.div(maxPower),
                    wheelVelocities.rightBack.div(maxPower),
                    wheelVelocities.rightFront.div(maxPower)
            );
        }

        // Apply feedforward
        MotorFeedforward feedforward = new MotorFeedforward(kStatic, kV, kA);
        double voltage = voltageSensor.getVoltage();
        double leftFrontPower = feedforward.compute(wheelVelocities.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVelocities.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVelocities.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVelocities.rightFront) / voltage;

        // Set motor powers
        frontLeft.setPower(leftFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
        frontRight.setPower(rightFrontPower);
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        poseEstimate = poseEstimate.plus(twist.value());
        return twist.velocity().value();
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void setPoseEstimate(Pose2d pose) {
        this.poseEstimate = pose;
    }

    private static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
    public ThreeWheelOdometryLocalizer getLocalizer() {
        return localizer;
    }
}