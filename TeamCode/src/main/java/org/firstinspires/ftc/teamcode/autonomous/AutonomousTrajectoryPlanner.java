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

    public TrajectorySequence buildSampleTrajectory() {
        TrajectorySequenceBuilder builder = driveTrain.trajectorySequenceBuilder(FieldMap.STARTING_POSITION);

        builder.forward(24)
                .strafeLeft(12)
                .lineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)))
                .waitSeconds(1)
                .back(24);

        return builder.build();
    }

    public void shutdown() {
        visionSystem.close();
    }
}