// File: AutonomousOpModeV2.java
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.robot.Robot;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.FieldConstants.TeamColor;
import org.firstinspires.ftc.teamcode.util.FieldConstants.StartingPosition;

/**
 * Autonomous Op Mode that allows pre-match configuration via Gamepad 1.
 */
@Autonomous(name = "Autonomous Op Mode V2")
public class AutonomousOpModeV2 extends LinearOpMode {

    private TeamColor teamColor = TeamColor.BLUE;
    private StartingPosition startingPosition = StartingPosition.LEFT;
    Pose2d startPose = FieldConstants.getStartingPose(teamColor, startingPosition);


    private Robot robot;
    private DriveTrainRR driveTrain;

    // Add this boolean to control waiting for user input
    private boolean waitForUserInput = true; // Set to false to disable waiting for 'A' between steps

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, true); // Pass true for isAutonomous
        driveTrain = new DriveTrainRR(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            handleMenuSelection();
            telemetry.addData("Team Color", teamColor);
            telemetry.addData("Starting Position", startingPosition);
            telemetry.update();
            sleep(100); // Prevent button spamming
        }

        driveTrain.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        if (startingPosition == StartingPosition.LEFT) {
            executeLeftStartingPosition();
        } else {
            executeRightStartingPosition();
        }

        // Ensure the robot keeps updating after the main tasks
        while (opModeIsActive()) {
            robot.update();
            driveTrain.update();
        }
    }

    private void handleMenuSelection() {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadLeft = gamepad1.dpad_left;

        if (dpadUp) {
            teamColor = (teamColor == TeamColor.RED) ? TeamColor.BLUE : TeamColor.RED;
        }

        if (dpadLeft) {
            startingPosition = (startingPosition == StartingPosition.LEFT) ? StartingPosition.RIGHT : StartingPosition.LEFT;
        }
    }

    private void executeLeftStartingPosition() {
        Pose2d endPose;
        Pose2d netPosition = FieldConstants.getNetPosition(teamColor);

        logAndWait("Driving to Net Position", netPosition);
        endPose = driveToPosition(netPosition, startPose);

        logAndWait("Scoring Preloaded Specimen", null);
        robot.setState(robot.scoringBasketState);
        waitForRobotStateCompletion();

        Pose2d[] samplePositions = FieldConstants.getSamplePositions(teamColor);

        for (Pose2d samplePosition : samplePositions) {
            logAndWait("Driving to Sample Position", samplePosition);
            endPose = driveToPosition(samplePosition, endPose);

            logAndWait("Picking Up Sample", null);
            robot.setState(robot.pickupState);
            waitForRobotStateStep("WAIT_FOR_DRIVE_FORWARD");

            logAndWait("Driving Forward to Pick Up Sample", null);
            endPose = driveForward(4, endPose);

            logAndWait("Completing Pickup", null);
            robot.pickupState.onDriveForwardComplete();
            waitForRobotStateCompletion();

            logAndWait("Returning to Net Position", netPosition);
            endPose = driveToPosition(netPosition, endPose);

            logAndWait("Scoring Sample", null);
            robot.setState(robot.scoringBasketState);
            waitForRobotStateCompletion();
        }

        Pose2d ascentZonePosition = FieldConstants.getAscentZonePosition(teamColor);
        logAndWait("Driving to Ascent Zone Position", ascentZonePosition);
        endPose = driveToPosition(ascentZonePosition, endPose);

        logAndWait("Starting Level One Ascent", null);
        robot.setState(robot.levelOneAscentState);
        waitForRobotStateCompletion();
    }

    private void executeRightStartingPosition() {
        telemetry.addLine("Right Starting Position Not Yet Implemented.");
        telemetry.update();
    }

    private Pose2d driveToPosition(Pose2d target, Pose2d start) {
        telemetry.addLine("Building Trajectory to Position...");
        telemetry.addData("Target Position", target);
        telemetry.addData("Current Position", driveTrain.getPoseEstimate());
        telemetry.update();

        TrajectorySequence sequence = driveTrain.trajectorySequenceBuilder(start)
                .lineToLinearHeading(target)
                .build();

        driveTrain.followTrajectorySequence(sequence);

        telemetry.addLine("Trajectory Complete.");
        telemetry.addData("Final Position", driveTrain.getPoseEstimate());
        telemetry.update();
        return sequence.end();
    }

    private Pose2d driveForward(double distance, Pose2d start) {
        telemetry.addLine("Building Forward Trajectory...");
        telemetry.addData("Distance", distance);
        telemetry.addData("Current Position", driveTrain.getPoseEstimate());
        telemetry.update();

        TrajectorySequence forwardSequence = driveTrain.trajectorySequenceBuilder(start)
                .forward(distance)
                .build();

        driveTrain.followTrajectorySequence(forwardSequence);

        telemetry.addLine("Forward Trajectory Complete.");
        telemetry.addData("Final Position", driveTrain.getPoseEstimate());
        telemetry.update();
        return forwardSequence.end();
    }

    private void logAndWait(String message, Pose2d position) {
        telemetry.addLine(message);
        if (position != null) {
            telemetry.addData("Target Position", position);
        }
        telemetry.addData("Current Position", driveTrain.getPoseEstimate());

        if (waitForUserInput) {
            telemetry.addLine("Press 'A' to Continue...");
            telemetry.update();

            while (!gamepad1.a && opModeIsActive()) {
                sleep(50); // Wait for button press
            }

            telemetry.addLine("Continuing...");
        } else {
            telemetry.update();
            sleep(500); // Optional brief pause to allow telemetry to update
        }
        telemetry.update();
    }

    private void waitForRobotStateCompletion() {
        while (opModeIsActive() && !robot.isCurrentStateCompleted()) {
            telemetry.addLine("Waiting for Robot State to Complete...");
            telemetry.addData("Current State", robot.getCurrentStateName());
            telemetry.addData("Current Step", robot.getCurrentSubstate());
            telemetry.update();
            robot.update();
            driveTrain.update();
        }
        telemetry.addLine("State Completed.");
        telemetry.update();
    }

    private void waitForRobotStateStep(String expectedStep) {
        while (opModeIsActive() && !robot.getCurrentSubstate().equals(expectedStep)) {
            telemetry.addLine("Waiting for Robot State Step...");
            telemetry.addData("Expected Step", expectedStep);
            telemetry.addData("Current Step", robot.getCurrentSubstate());
            telemetry.update();
            robot.update();
            driveTrain.update();
        }
        telemetry.addLine("Expected Step Reached.");
        telemetry.update();
    }
}