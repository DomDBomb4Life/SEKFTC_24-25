// File: DriveTrainRR.java
package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.localization.ThreeWheelOdometryLocalizer;

public class DriveTrainRR {

    // Motors
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

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
    //private final BNO055IMU imu;

    // Flags for asynchronous operations
    private boolean isBusy = false;

    // Target heading for turnAsync
    private double targetHeading;

    // Trajectory following variables
    private TimeTrajectory currentTrajectory;
    private double trajectoryStartTime;

    public DriveTrainRR(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "FrontL");
        backLeft = hardwareMap.get(DcMotor.class, "BackL");
        frontRight = hardwareMap.get(DcMotor.class, "FrontR");
        backRight = hardwareMap.get(DcMotor.class, "BackR");

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
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        //BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        //imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //imu.initialize(imuParameters);

        // Initialize localizer
        localizer = new ThreeWheelOdometryLocalizer(hardwareMap);

        // Initialize pose estimate
        poseEstimate = new Pose2d(new Vector2d(0, 0), Rotation2d.exp(0));
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

    // Added method to run actions
    public void runAction(Action action) {
        TelemetryPacket packet = new TelemetryPacket();
        while (action.run(packet)) {
            update();
            // Additional updates or telemetry can be added here
        }
    }

    // Implemented turnAsync method
    public void turnAsync(double angle) {
        targetHeading = poseEstimate.heading.log() + angle;
        isBusy = true;
    }

    // Implemented followTrajectoryAsync method
    public void followTrajectoryAsync(TimeTrajectory trajectory) {
        currentTrajectory = trajectory;
        trajectoryStartTime = System.currentTimeMillis() / 1000.0;
        isBusy = true;
    }

    // Implemented update method
    public void update() {
        updatePoseEstimate();

        if (isBusy) {
            if (currentTrajectory != null) {
                double currentTime = System.currentTimeMillis() / 1000.0 - trajectoryStartTime;
                if (currentTime > currentTrajectory.duration) {
                    setDrivePower(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    isBusy = false;
                    currentTrajectory = null;
                } else {
                    Pose2d targetPose = currentTrajectory.get(currentTime).value();
                    Pose2d currentPose = getPoseEstimate();

                    // Compute the error between current pose and target pose
                    Vector2d positionError = targetPose.position.minus(currentPose.position);
//                    double headingError = targetPose.heading.minus(currentPose.heading).log();

                    // Simple proportional controller
                    double kP = 1.0;
                    Vector2d velocity = positionError.times(kP);
//                    double angularVelocity = headingError * kP;

//                    setDrivePower(new PoseVelocity2d(velocity, angularVelocity));
                }
            } else {
                // Turning logic
                //double currentHeading = imu.getAngularOrientation().firstAngle;
                //double error = targetHeading - currentHeading;

                // Simple proportional control for turning
                double kP = 0.5;
                //double turnPower = kP * error;
/*
                if (Math.abs(error) < 0.01) { // Threshold for completion
                    setDrivePower(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    isBusy = false;
                } else {
                    setDrivePower(new PoseVelocity2d(new Vector2d(0, 0), turnPower));
                }

 */
            }
        }
    }

    // Implemented isBusy method
    public boolean isBusy() {
        return isBusy;
    }
}