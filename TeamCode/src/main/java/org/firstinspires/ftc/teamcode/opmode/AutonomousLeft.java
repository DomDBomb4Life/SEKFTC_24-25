package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.FieldConstants;

/**
 * Autonomous for the LEFT starting position.
 * Removed team color logic and menu selection.
 * Uses fixed positions from FieldConstants.
 */
@Autonomous(name = "Autonomous Left Start")
public class AutonomousLeft extends OpMode {
    private Robot robot;
    private DriveTrainRR driveTrain;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true);
        driveTrain = new DriveTrainRR(hardwareMap);

        // Set initial pose for LEFT start
        driveTrain.setPoseEstimate(FieldConstants.LEFT_START);

        telemetry.addLine("Initialized Left Auto");
    }

    @Override
    public void start() {
        // Build trajectory for LEFT start
        TrajectorySequenceBuilder seqBuilder = driveTrain.trajectorySequenceBuilder(FieldConstants.LEFT_START);

        // Drive to net
        seqBuilder.lineToLinearHeading(FieldConstants.NET_POSITION);

        // Score preloaded specimen
        seqBuilder.addTemporalMarkerOffset(-1, () -> {
            robot.setState(robot.scoringBasketState);
        });
        seqBuilder.waitSeconds(5);

        // For each sample position
        for (Pose2d samplePosition : FieldConstants.SAMPLE_POSITIONS) {
            seqBuilder.lineToLinearHeading(samplePosition);

            // Simulate pickup process
            seqBuilder.addTemporalMarkerOffset(-0.5, () -> {
                robot.setState(robot.homeState);
            });

            seqBuilder.waitSeconds(1.0);
            // Simulate driving forward and closing claw
            seqBuilder.addTemporalMarkerOffset(0.0,() -> {
                robot.onRightTriggerPressed();
            });

            seqBuilder.waitSeconds(1.0);

            // Return to net
            seqBuilder.lineToLinearHeading(FieldConstants.NET_POSITION);
            seqBuilder.addTemporalMarkerOffset(-1, () -> {
                robot.setState(robot.scoringBasketState);
            });
            seqBuilder.waitSeconds(5);
        }

        // Move to ascent zone
        seqBuilder.addTemporalMarkerOffset(0, () -> {
            robot.setState(robot.levelOneAscentState);
        });

        seqBuilder.lineToLinearHeading(FieldConstants.ASCENT_ZONE_POSITION);
        seqBuilder.waitSeconds(1);
        seqBuilder.forward(4);

        seqBuilder.addTemporalMarkerOffset(0.0,() -> {
            robot.onRightTriggerPressed();
        });

        TrajectorySequence sequence = seqBuilder.build();
        driveTrain.followTrajectorySequenceAsync(sequence);
    }

    @Override
    public void loop() {
        driveTrain.update();
        robot.update();

        telemetry.addData("Current Pose", driveTrain.getPoseEstimate());
        telemetry.addData("Current Robot State", robot.getCurrentStateName());
        telemetry.addData("Current Robot Substate", robot.getCurrentSubstate());
        telemetry.update();
    }
}