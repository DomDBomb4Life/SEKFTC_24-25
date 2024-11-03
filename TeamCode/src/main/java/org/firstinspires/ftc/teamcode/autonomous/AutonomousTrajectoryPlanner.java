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
import org.firstinspires.ftc.teamcode.localization.ThreeWheelOdometryLocalizer;
// import org.firstinspires.ftc.teamcode.vision.VisionSystem; // Commented out

public class AutonomousTrajectoryPlanner {

    private final DriveTrainRR driveTrain;
    // private final VisionSystem visionSystem; // Commented out
    private final ThreeWheelOdometryLocalizer localizer;

    public AutonomousTrajectoryPlanner(HardwareMap hardwareMap) {
        driveTrain = new DriveTrainRR(hardwareMap);
        // visionSystem = new VisionSystem(hardwareMap); // Commented out
        localizer = driveTrain.getLocalizer();
    }

    public void initialize() {
        // Set the initial pose estimate
        driveTrain.setPoseEstimate(FieldMap.STARTING_POSITION);
        // visionSystem.initialize(); // Commented out
    }

    public void updateLocalization() {
        // Update odometry
        driveTrain.updatePoseEstimate();

        // Periodically update the pose estimate with vision data
    
    }

    // Commented out because vision system is not implemented
    // private Pose2d fusePoses(Pose2d odometryPose, Pose2d visionPose) {
    //     // Simple fusion: weighted average
    //     double alpha = 0.8; // Weight for vision
    //     double beta = 1 - alpha; // Weight for odometry
    //
    //     double fusedX = alpha * visionPose.position.x + beta * odometryPose.position.x;
    //     double fusedY = alpha * visionPose.position.y + beta * odometryPose.position.y;
    //     double fusedHeading = alpha * visionPose.heading.getRadians() + beta * odometryPose.heading.getRadians();
    //
    //     return new Pose2d(new Vector2d(fusedX, fusedY), fusedHeading);
    // }

    public Action buildAutonomousSequence() {
        // Initialize the TrajectoryActionBuilder

        // Create the necessary parameters for TrajectoryActionBuilder
        double arcLengthSamplingEps = 0.1; // Tuning parameter for path sampling
        ProfileParams profileParams = new ProfileParams(0.5, 0.5, 0.1); // Corrected constructor
        TrajectoryBuilderParams trajectoryBuilderParams = new TrajectoryBuilderParams(arcLengthSamplingEps, profileParams);

        double beginEndVel = 0.0; // Starting and ending velocity
        TurnConstraints baseTurnConstraints = new TurnConstraints(5.0, -10.0, 10.0); // Corrected constructor

        // Implement VelConstraint as a lambda expression
        VelConstraint baseVelConstraint = (robotPose, path, s) -> 50.0; // Max linear velocity

        // Implement AccelConstraint as a lambda expression
        AccelConstraint baseAccelConstraint = (robotPose, path, s) -> new MinMax(-30.0, 30.0); // Min and Max acceleration

        PoseMap poseMap = new IdentityPoseMap(); // Default pose mapping

        TrajectoryActionBuilder actionBuilder = new TrajectoryActionBuilder(
                (TurnActionFactory) new TurnActionFactoryImpl(driveTrain),
                (TrajectoryActionFactory) new TrajectoryActionFactoryImpl(driveTrain),
                trajectoryBuilderParams,
                driveTrain.getPoseEstimate(),
                beginEndVel,
                baseTurnConstraints,
                baseVelConstraint,
                baseAccelConstraint,
                poseMap
        );

        // Build the action sequence using advanced trajectory features
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

    public void executeAutonomousSequence(Action actionSequence) {
        // Execute the action sequence
        driveTrain.runAction(actionSequence);
    }

    public void shutdown() {
        // visionSystem.close(); // Commented out
    }
}