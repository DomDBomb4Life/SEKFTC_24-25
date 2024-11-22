// File: DriveTrainRR.java
package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.*;
import com.acmerobotics.roadrunner.followers.*;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.localization.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.*;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

/**
 * DriveTrain subsystem using RoadRunner for trajectory following.
 */
public class DriveTrainRR extends MecanumDrive {
    // Hardware components
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

        // Set motor modes and directions
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setVelocityPIDFCoefficients(
                        MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        // Reverse motors if necessary
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
        // Return the heading from an external sensor if available
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // Return the heading velocity from an external sensor if available
        return 0.0;
    }

    @Override
    public List<Double> getWheelPositions() {
        // Return the current positions of the drive motors in inches
        return Arrays.asList(
                encoderTicksToInches(leftFront.getCurrentPosition()),
                encoderTicksToInches(leftRear.getCurrentPosition()),
                encoderTicksToInches(rightRear.getCurrentPosition()),
                encoderTicksToInches(rightFront.getCurrentPosition())
        );
    }

    @Override
    public List<Double> getWheelVelocities() {
        // Return the current velocities of the drive motors in inches per second
        return Arrays.asList(
                encoderTicksToInches(leftFront.getVelocity()),
                encoderTicksToInches(leftRear.getVelocity()),
                encoderTicksToInches(rightRear.getVelocity()),
                encoderTicksToInches(rightFront.getVelocity())
        );
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

    // Trajectory builders
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

    // Constraints for trajectory generation
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

    // Manual control method with power weighting
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

    // Methods for turning
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
}