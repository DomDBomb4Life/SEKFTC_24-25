// File: AutonomousTrajectoryPlanner.java
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.TurnActionFactory;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.field.FieldMap;
import org.firstinspires.ftc.teamcode.localization.ThreeWheelOdometryLocalizer;
import org.firstinspires.ftc.teamcode.vision.VisionSystem;

public class AutonomousTrajectoryPlanner {

    private final DriveTrainRR driveTrain;
    private final VisionSystem visionSystem;
    private final ThreeWheelOdometryLocalizer localizer;

    public AutonomousTrajectoryPlanner(HardwareMap hardwareMap) {
        driveTrain = new DriveTrainRR(hardwareMap);
        visionSystem = new VisionSystem(hardwareMap);
        localizer = driveTrain.getLocalizer();
    }

    public void initialize() {
        // Set the initial pose estimate
        driveTrain.setPoseEstimate(FieldMap.STARTING_POSITION);
        visionSystem.initialize();
    }

    public void updateLocalization() {
        // Update odometry
        driveTrain.updatePoseEstimate();

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

    public Action buildAutonomousSequence() {
        // Initialize the TrajectoryActionBuilder
        TrajectoryActionBuilder actionBuilder = new TrajectoryActionBuilder(
                new TurnActionFactoryImpl(driveTrain),
                new TrajectoryActionFactoryImpl(driveTrain),
                driveTrain.getPoseEstimate(),
                0.0 // Begin end velocity
        );

        // Build the action sequence using advanced trajectory features
        actionBuilder
                .splineTo(FieldMap.ALLIANCE_SAMPLE_1.vec(), FieldMap.ALLIANCE_SAMPLE_1.getHeading())
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up sample 1
                    // Example: robot.claw.close();
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.ALLIANCE_SAMPLE_2.vec(), FieldMap.ALLIANCE_SAMPLE_2.getHeading())
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up sample 2
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.ALLIANCE_SAMPLE_3.vec(), FieldMap.ALLIANCE_SAMPLE_3.getHeading())
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up sample 3
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.OBSERVATION_AREA.vec(), FieldMap.OBSERVATION_AREA.getHeading())
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to drop off samples
                    // Example: robot.claw.open();
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.YELLOW_SAMPLE_1.vec(), FieldMap.YELLOW_SAMPLE_1.getHeading())
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up yellow sample 1
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.BASKET_POSITION.vec(), FieldMap.BASKET_POSITION.getHeading())
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to score yellow sample 1
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.YELLOW_SAMPLE_2.vec(), FieldMap.YELLOW_SAMPLE_2.getHeading())
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to pick up yellow sample 2
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.BASKET_POSITION.vec(), FieldMap.BASKET_POSITION.getHeading())
                .afterTime(0.0, new InstantAction(() -> {
                    // Code to score yellow sample 2
                }))
                .waitSeconds(0.5)
                .splineTo(FieldMap.LOW_RUNG_POSITION.vec(), FieldMap.LOW_RUNG_POSITION.getHeading())
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
        visionSystem.close();
    }
}