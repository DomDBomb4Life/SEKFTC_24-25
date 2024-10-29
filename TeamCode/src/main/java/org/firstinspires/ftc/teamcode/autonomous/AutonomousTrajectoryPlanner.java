// File: AutonomousTrajectoryPlanner.java
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.sequence.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.field.FieldMap;
import org.firstinspires.ftc.teamcode.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.localization.ThreeWheelOdometryLocalizer;

public class AutonomousTrajectoryPlanner {

    private final DriveTrainRR driveTrain;
    private final VisionSystem visionSystem;
    private final ThreeWheelOdometryLocalizer localizer;

    public AutonomousTrajectoryPlanner(HardwareMap hardwareMap) {
        driveTrain = new DriveTrainRR(hardwareMap);
        visionSystem = new VisionSystem(hardwareMap);
        localizer = (ThreeWheelOdometryLocalizer) driveTrain.getLocalizer();
    }

    public void initialize() {
        // Set the initial pose estimate
        driveTrain.setPoseEstimate(FieldMap.STARTING_POSITION);
    }

    public void updateLocalization() {
        // Update odometry
        driveTrain.update();

        // Periodically update the pose estimate with vision data
        visionSystem.update();
        Pose2d visionPose = visionSystem.getAprilTagPose();

        if (visionPose != null) {
            // Fuse the vision pose with the current pose estimate
            Pose2d currentPose = driveTrain.getPoseEstimate();
            Pose2d fusedPose = fusePoses(currentPose, visionPose);
            driveTrain.setPoseEstimate(fusedPose);
        }
    }

    private Pose2d fusePoses(Pose2d odometryPose, Pose2d visionPose) {
        // Simple fusion: weighted average
        double alpha = 0.8; // Weight for vision
        double beta = 1 - alpha; // Weight for odometry

        double fusedX = alpha * visionPose.getX() + beta * odometryPose.getX();
        double fusedY = alpha * visionPose.getY() + beta * odometryPose.getY();
        double fusedHeading = alpha * visionPose.getHeading() + beta * odometryPose.getHeading();

        return new Pose2d(fusedX, fusedY, fusedHeading);
    }

    public void executeTrajectorySequence(TrajectorySequence trajectorySequence) {
        driveTrain.followTrajectorySequence(trajectorySequence);
    }

    // Build trajectory to pick up alliance samples and place them in the observation area
    public TrajectorySequence buildAllianceSampleTrajectory() {
        TrajectorySequenceBuilder builder = driveTrain.trajectorySequenceBuilder(FieldMap.STARTING_POSITION);

        // Move to each alliance sample and pick it up
        builder.lineToLinearHeading(FieldMap.ALLIANCE_SAMPLE_1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Code to pick up sample 1
                    // Example: robot.claw.close();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(FieldMap.ALLIANCE_SAMPLE_2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Code to pick up sample 2
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(FieldMap.ALLIANCE_SAMPLE_3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Code to pick up sample 3
                })
                .waitSeconds(0.5)
                // Move to the observation area to drop off samples
                .lineToLinearHeading(FieldMap.OBSERVATION_AREA)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Code to drop off samples
                    // Example: robot.claw.open();
                })
                .waitSeconds(0.5);

        return builder.build();
    }

    // Build trajectory to collect yellow samples and score them in the basket
    public TrajectorySequence buildYellowSampleScoringTrajectory() {
        TrajectorySequenceBuilder builder = driveTrain.trajectorySequenceBuilder(FieldMap.OBSERVATION_AREA);

        // Move to yellow sample 1
        builder.lineToLinearHeading(FieldMap.YELLOW_SAMPLE_1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Code to pick up yellow sample 1
                })
                .waitSeconds(0.5)
                // Move to basket to score
                .lineToLinearHeading(FieldMap.BASKET_POSITION)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Code to score yellow sample 1
                })
                .waitSeconds(0.5)
                // Repeat for yellow sample 2
                .lineToLinearHeading(FieldMap.YELLOW_SAMPLE_2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Code to pick up yellow sample 2
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(FieldMap.BASKET_POSITION)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Code to score yellow sample 2
                })
                .waitSeconds(0.5);

        return builder.build();
    }

    // Build trajectory to move to low rung position for level 1 ascent
    public TrajectorySequence buildLevelOneAscentTrajectory() {
        TrajectorySequenceBuilder builder = driveTrain.trajectorySequenceBuilder(FieldMap.BASKET_POSITION);

        builder.lineToLinearHeading(FieldMap.LOW_RUNG_POSITION)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Code to initiate level 1 ascent
                    // Example: robot.viperLift.liftToPosition(levelOneHeight);
                })
                .waitSeconds(0.5);

        return builder.build();
    }

    public void shutdown() {
        visionSystem.close();
    }
}