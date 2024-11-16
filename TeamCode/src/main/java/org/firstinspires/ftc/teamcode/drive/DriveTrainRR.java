// File: DriveTrainRR.java
package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.*;
import com.acmerobotics.roadrunner.followers.*;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.localization.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.*;

import java.util.Arrays;
import java.util.List;

/**
 * DriveTrain subsystem using RoadRunner for trajectory following.
 */
@Config
public class DriveTrainRR extends MecanumDrive {
    // PID coefficients can be adjusted in the dashboard
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.0;

    public static double VX_WEIGHT = 1.0;
    public static double VY_WEIGHT = 1.0;
    public static double OMEGA_WEIGHT = 1.0;

    // Motor constants
    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;

    // Physical constants
    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 8.839; // in

    // Feedforward parameters
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0.0;
    public static double kStatic = 0.0;

    // Constraints
    public static double MAX_VEL = rpmToVelocity(MAX_RPM);
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private VoltageSensor batteryVoltageSensor;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private HolonomicPIDVAFollower follower;

    public DriveTrainRR(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, LATERAL_MULTIPLIER);

        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "FrontL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BackL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BackR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FrontR");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Reverse motors if necessary (make configurable)
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Set up the localizer
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        // Initialize the trajectory follower
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);

        // Initialize the trajectory sequence runner
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return 0.0;
    }

    @Override
    public List<Double> getWheelPositions() {
        // Since motors don't have encoders, return zeros
        return Arrays.asList(0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public List<Double> getWheelVelocities() {
        // Since motors don't have encoders, return zeros
        return Arrays.asList(0.0, 0.0, 0.0, 0.0);
    }

    public void update() {
        updatePoseEstimate();

        Pose2d poseEstimate = getPoseEstimate();

        DriveSignal signal = trajectorySequenceRunner.update(poseEstimate, getPoseVelocity());

        if (signal != null) {
            setDriveSignal(signal);
        }
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence sequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(sequence);
    }

    public void followTrajectorySequence(TrajectorySequence sequence) {
        followTrajectorySequenceAsync(sequence);
        while (isBusy()) {
            update();
        }
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                getAccelerationConstraint(MAX_ACCEL),
                MAX_ANG_VEL,
                MAX_ANG_ACCEL
        );
    }

    // Added methods
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(
                startPose,
                getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                getAccelerationConstraint(MAX_ACCEL)
        );
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(
                startPose,
                reversed,
                getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                getAccelerationConstraint(MAX_ACCEL)
        );
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(
                Arrays.asList(
                        new AngularVelocityConstraint(maxAngularVel),
                        new MecanumVelocityConstraint(maxVel, trackWidth)
                )
        );
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    // Added method for manual control
    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;
    
        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.getHeading()) > 1) {
            // Normalize the powers according to the weights
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

    // Added method for turning
    public void turnAsync(double angle) {
        TrajectorySequence turnSequence = trajectorySequenceBuilder(getPoseEstimate())
                .turn(angle)
                .build();
        followTrajectorySequenceAsync(turnSequence);
    }

    public void turn(double angle) {
        turnAsync(angle);
        while (isBusy()) {
            update();
        }
    }

    // Helper method to compute RPM to velocity
    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
}