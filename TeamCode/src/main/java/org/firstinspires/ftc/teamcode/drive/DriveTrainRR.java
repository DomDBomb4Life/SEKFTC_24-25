package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.*;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.*;
import com.acmerobotics.roadrunner.followers.*;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.localization.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.*;

import java.util.Arrays;
import java.util.List;

/**
 * DriveTrain subsystem using RoadRunner for trajectory following.
 */
@Config
public class DriveTrainRR extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.0;

    public static double VX_WEIGHT = 1.0;
    public static double VY_WEIGHT = 1.0;
    public static double OMEGA_WEIGHT = 1.0;

    public static double MAX_VEL = 5.0; // inches per second
    public static double MAX_ACCEL = 30.0; // inches per second squared
    public static double MAX_ANG_VEL = Math.toRadians(180); // radians per second
    public static double MAX_ANG_ACCEL = Math.toRadians(180); // radians per second squared
    public static double TRACK_WIDTH = 8.5; // inches

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private VoltageSensor batteryVoltageSensor;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    public DriveTrainRR(HardwareMap hardwareMap) {
        super(1.0, 1.0, 1.0, LATERAL_MULTIPLIER);

        leftFront = hardwareMap.get(DcMotorEx.class, "FrontL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BackL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BackR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FrontR");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Reverse motors if necessary
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Set up the localizer
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        // Initialize the trajectory follower
        TrajectoryFollower follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);

        // Initialize the trajectory sequence runner
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID, batteryVoltageSensor);
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
        // If you have a gyroscope, return its heading here
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // If you have a gyroscope, return its angular velocity here
        return 0.0;
    }

    @Override
    public List<Double> getWheelPositions() {
        // Return the positions of the wheels if needed
        return Arrays.asList(
                (double) leftFront.getCurrentPosition(),
                (double) leftRear.getCurrentPosition(),
                (double) rightRear.getCurrentPosition(),
                (double) rightFront.getCurrentPosition()
        );
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate());
        if (signal != null) {
            setDriveSignal(signal);
        }
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void followTrajectorySequence(TrajectorySequence sequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(sequence);
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
}