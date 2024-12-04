// File: AutonomousOpModeV2.java
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.robot.Robot;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.FieldConstants.TeamColor;
import org.firstinspires.ftc.teamcode.util.FieldConstants.StartingPosition;

/**
 * Autonomous Op Mode that allows pre-match configuration via Gamepad 1.
 */
@Autonomous(name = "Autonomous Op Mode V2")
public class AutonomousOpModeV3 extends OpMode {

    private TeamColor teamColor = TeamColor.BLUE;
    private StartingPosition startingPosition = StartingPosition.LEFT;

    private Robot robot;
    private DriveTrainRR driveTrain;

    private boolean isInitialized = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true); // Pass true for isAutonomous
        driveTrain = new DriveTrainRR(hardwareMap);

        // Initialize pose estimate
        Pose2d startPose = FieldConstants.getStartingPose(teamColor, startingPosition);
        driveTrain.setPoseEstimate(startPose);

        telemetry.addLine("Initialized");
    }

    @Override
    public void init_loop() {
        handleMenuSelection();
        telemetry.addData("Team Color", teamColor);
        telemetry.addData("Starting Position", startingPosition);
        telemetry.update();
    }

    @Override
    public void start() {
        if (startingPosition == StartingPosition.LEFT) {
            executeLeftStartingPosition();
        } else {
            executeRightStartingPosition();
        }
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

    private void handleMenuSelection() {
        if (gamepad1.dpad_up) {
            teamColor = (teamColor == TeamColor.RED) ? TeamColor.BLUE : TeamColor.RED;
        }

        if (gamepad1.dpad_left) {
            startingPosition = (startingPosition == StartingPosition.LEFT) ? StartingPosition.RIGHT : StartingPosition.LEFT;
        }
    }

    private void executeLeftStartingPosition() {
        Pose2d startPose = FieldConstants.getStartingPose(teamColor, startingPosition);
        Pose2d netPosition = FieldConstants.getNetPosition(teamColor);
        Pose2d ascentZonePosition = FieldConstants.getAscentZonePosition(teamColor);
        Pose2d[] samplePositions = FieldConstants.getSamplePositions(teamColor);

        // Build the trajectory sequence
        TrajectorySequenceBuilder seqBuilder = driveTrain.trajectorySequenceBuilder(startPose);

        // Drive to Net Position
        seqBuilder.lineToLinearHeading(netPosition);

        // Add marker to set robot state to scoringBasketState
        seqBuilder.addTemporalMarkerOffset(-1, () -> {
            robot.setState(robot.scoringBasketState);
        });
        seqBuilder.waitSeconds(5);

        // For each sample position
        for (Pose2d samplePosition : samplePositions) {
            // Drive to Sample Position
            seqBuilder.lineToLinearHeading(samplePosition);

            // Add marker to set robot state to pickupState
            seqBuilder.addTemporalMarkerOffset(0, () -> {
                robot.setState(robot.pickupState);
            });

            // Wait for robot to reach "WAIT_FOR_DRIVE_FORWARD"
            // Adjust timing as necessary
            seqBuilder.waitSeconds(2.0);

            // Drive forward 4 inches
            seqBuilder.forward(2);

            // Add marker to call robot.pickupState.onDriveForwardComplete()
            seqBuilder.addTemporalMarkerOffset(0.5,() -> {
                robot.pickupState.onDriveForwardComplete();
            });

            // Wait for robot state completion
            seqBuilder.waitSeconds(0.5);

            // Drive back to Net Position
            seqBuilder.lineToLinearHeading(netPosition);

            // Add marker to set robot state to scoringBasketState
            seqBuilder.addTemporalMarkerOffset(-1, () -> {
                robot.setState(robot.scoringBasketState);
            });
            seqBuilder.waitSeconds(4);
        }

        // Drive to Ascent Zone Position
        seqBuilder.lineToLinearHeading(ascentZonePosition);

        // Add marker to set robot state to levelOneAscentState
        seqBuilder.addTemporalMarker(() -> {
            robot.setState(robot.levelOneAscentState);
        });

        // Build the trajectory sequence
        TrajectorySequence sequence = seqBuilder.build();

        // Start following the trajectory sequence asynchronously
        driveTrain.followTrajectorySequenceAsync(sequence);
    }

    private void executeRightStartingPosition() {
        telemetry.addLine("Right Starting Position Not Yet Implemented.");
        telemetry.update();
    }

    // Helper method to simulate sleep in OpMode

}