// File: DriveTrainRR.java
package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;

import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.localization.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;


@Config
public class DriveTrainRR extends MecanumDrive {

    // Motors
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    // IMU

    // Voltage sensor
    private VoltageSensor batteryVoltageSensor;

    // Trajectory sequence runner
    private TrajectorySequenceRunner trajectorySequenceRunner;
    private final TrajectoryFollower follower;


    private final List<Integer> lastEncoderPositions = new ArrayList<>();
    private final List<Integer> lastEncoderVelocities = new ArrayList<>();

    public DriveTrainRR(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, LATERAL_MULTIPLIER);

        // Initialize voltage sensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Ensure the robot's configuration is up to date
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // Set bulk caching mode for better performance
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize IMU


        // Initialize motors with your configuration names
        leftFront = hardwareMap.get(DcMotorEx.class, "FrontL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BackL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BackR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FrontR");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        // Set motor modes and directions
        for (DcMotorEx motor : motors) {
            // Ensure motors are in the correct configuration
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Reverse motors if necessary
        // Match your original motor directions
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // If using encoders
        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (MOTOR_VELO_PID != null) {
                setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
            }
        } else {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        List<Integer> lastTrackingEncoderPositions = new ArrayList<>();
        List<Integer> lastTrackingEncoderVelocities = new ArrayList<>();

        // Set the localizer (e.g., if using tracking wheels)
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncoderPositions, lastTrackingEncoderVelocities));

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        // Initialize trajectory sequence runner
        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncoderPositions, lastEncoderVelocities, lastTrackingEncoderPositions, lastTrackingEncoderVelocities
        );
    }

    // Trajectory builder methods
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL));
    }



    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                getAccelerationConstraint(MAX_ACCEL),
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    // Trajectory following methods
    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }


    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void update() {
        updatePoseEstimate();

        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());

        if (signal != null) {
            setDriveSignal(signal);
        }
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftFront.getCurrentPosition()),
                encoderTicksToInches(leftRear.getCurrentPosition()),
                encoderTicksToInches(rightRear.getCurrentPosition()),
                encoderTicksToInches(rightFront.getCurrentPosition())
        );
    }

    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftFront.getVelocity()),
                encoderTicksToInches(leftRear.getVelocity()),
                encoderTicksToInches(rightRear.getVelocity()),
                encoderTicksToInches(rightFront.getVelocity())
        );
    }

    @Override
    public double getRawExternalHeading() {
        // Assuming the IMU's heading is in radians
        return 0.0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // Assuming the IMU provides angular velocity in radians per second
        return 0.0;
    }

    // Set motor modes
    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    // Set zero power behavior
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    // Set PIDF coefficients
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    // Set weighted drive power for teleop
    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // Re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    // Constraint methods
    public static TrajectoryVelocityConstraint getVelocityConstraint(
            double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}