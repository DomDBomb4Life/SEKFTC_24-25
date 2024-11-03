// File: AutonomousTrajectoryPlanner.java
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnActionFactory;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.field.FieldMap;
import org.firstinspires.ftc.teamcode.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.localization.ThreeWheelOdometryLocalizer;
import org.firstinspires.ftc.teamcode.vision.VisionSystem;

/**
 * The AutonomousTrajectoryPlanner class is responsible for planning and executing autonomous trajectories.
 * It integrates vision-based localization using AprilTags and fuses it with odometry data for accurate pose estimation.
 */
public class AutonomousTrajectoryPlanner {

    private final DriveTrainRR driveTrain;
    private final VisionSystem visionSystem;
    private final VisionLocalization visionLocalization;
    private final ThreeWheelOdometryLocalizer localizer;

    /**
     * Constructor to initialize subsystems.
     *
     * @param hardwareMap The hardware map provided by the FTC SDK.
     */
    public AutonomousTrajectoryPlanner(HardwareMap hardwareMap) {
        driveTrain = new DriveTrainRR(hardwareMap);
        visionSystem = new VisionSystem(hardwareMap);
        visionLocalization = new VisionLocalization(visionSystem);
        localizer = driveTrain.getLocalizer();
    }

    /**
     * Initializes the autonomous trajectory planner by setting the initial pose.
     * No additional initialization for VisionSystem is required as it's handled in the constructor.
     */
    public void initialize() {
        // Set the initial pose estimate using Roadrunner
        driveTrain.setPoseEstimate(FieldMap.STARTING_POSITION);
    }

    /**
     * Updates the robot's localization by fusing odometry and vision data.
     * This method should be called periodically during autonomous operation.
     */
    public void updateLocalization() {
        // Update odometry data
        driveTrain.updatePoseEstimate();

        // Retrieve vision-based pose estimate
        Pose2d visionPose = visionLocalization.getRobotPose();

        if (visionPose != null) {
            // Retrieve odometry-based pose estimate
            Pose2d odometryPose = driveTrain.getPoseEstimate();

            // Fuse the two pose estimates using a weighted average
            Pose2d fusedPose = fusePoses(odometryPose, visionPose);

            // Update Roadrunner's pose estimate with the fused pose
            driveTrain.setPoseEstimate(fusedPose);
        }
    }

    /**
     * Fuses odometry and vision pose estimates using a weighted average.
     *
     * @param odometryPose The pose estimate from odometry.
     * @param visionPose   The pose estimate from vision.
     * @return The fused Pose2d.
     */
    private Pose2d fusePoses(Pose2d odometryPose, Pose2d visionPose) {
        // Weighted average coefficients
        double alpha = 0.5; // Weight for vision
        double beta = 1 - alpha; // Weight for odometry

        double fusedX = alpha * visionPose.getX() + beta * odometryPose.getX();
        double fusedY = alpha * visionPose.getY() + beta * odometryPose.getY();
        double fusedHeading = alpha * visionPose.getHeading() + beta * odometryPose.getHeading();

        return new Pose2d(fusedX, fusedY, fusedHeading);
    }

    /**
     * Builds the autonomous action sequence using Roadrunner's TrajectoryActionBuilder.
     *
     * @return The constructed Action sequence.
     */
    public Action buildAutonomousSequence() {
        // Define trajectory parameters
        double arcLengthSamplingEps = 0.1; // Tuning parameter for path sampling
        ProfileParams profileParams = new ProfileParams(0.5, 0.5, 0.1);
        TrajectoryBuilderParams trajectoryBuilderParams = new TrajectoryBuilderParams(arcLengthSamplingEps, profileParams);

        double beginEndVel = 0.0; // Starting and ending velocity
        TurnConstraints baseTurnConstraints = new TurnConstraints(5.0, -10.0, 10.0);

        // Define velocity and acceleration constraints
        VelConstraint baseVelConstraint = (robotPose, path, s) -> 50.0; // Max linear velocity
        AccelConstraint baseAccelConstraint = (robotPose, path, s) -> new MinMax(-30.0, 30.0); // Min and Max acceleration

        PoseMap poseMap = new IdentityPoseMap(); // Default pose mapping

        // Initialize TrajectoryActionBuilder with necessary factories and parameters
        TrajectoryActionBuilder actionBuilder = new TrajectoryActionBuilder(
                new TurnActionFactoryImpl(driveTrain),
                new TrajectoryActionFactoryImpl(driveTrain),
                trajectoryBuilderParams,
                driveTrain.getPoseEstimate(),
                beginEndVel,
                baseTurnConstraints,
                baseVelConstraint,
                baseAccelConstraint,
                poseMap
        );

        // Build the action sequence using splines and actions
        actionBuilder
                .splineTo(FieldMap.ALLIANCE_SAMPLE_1.position, FieldMap.ALLIANCE_SAMPLE_1.heading)
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up sample 1
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.ALLIANCE_SAMPLE_2.position, FieldMap.ALLIANCE_SAMPLE_2.heading)
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up sample 2
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.ALLIANCE_SAMPLE_3.position, FieldMap.ALLIANCE_SAMPLE_3.heading)
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up sample 3
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.OBSERVATION_AREA.position, FieldMap.OBSERVATION_AREA.heading)
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to drop off samples
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.YELLOW_SAMPLE_1.position, FieldMap.YELLOW_SAMPLE_1.heading)
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up yellow sample 1
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.BASKET_POSITION.position, FieldMap.BASKET_POSITION.heading)
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to score yellow sample 1
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.YELLOW_SAMPLE_2.position, FieldMap.YELLOW_SAMPLE_2.heading)
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up yellow sample 2
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.BASKET_POSITION.position, FieldMap.BASKET_POSITION.heading)
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to score yellow sample 2
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.LOW_RUNG_POSITION.position, FieldMap.LOW_RUNG_POSITION.heading)
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to initiate level 1 ascent
                }))
                .waitSeconds(0.5);

        // Build and return the action sequence
        return actionBuilder.build();
    }

    /**
     * Executes the provided autonomous action sequence.
     *
     * @param actionSequence The Action sequence to execute.
     */
    public void executeAutonomousSequence(Action actionSequence) {
        // Execute the action sequence using Roadrunner's drivetrain
        driveTrain.runAction(actionSequence);
    }

    /**
     * Shuts down the autonomous trajectory planner, releasing resources.
     */
    public void shutdown() {
        // Properly close the vision system to release camera resources
        visionSystem.close();
    }
}