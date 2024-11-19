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

    private Robot robot;
    private DriveTrainRR driveTrain;

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

        Pose2d startPose = FieldConstants.getStartingPose(teamColor, startingPosition);
        driveTrain.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        if (startingPosition == StartingPosition.LEFT) {
            executeLeftStartingPosition();
        } else {
            executeRightStartingPosition();
        }
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
        Pose2d netPosition = FieldConstants.getNetPosition(teamColor);

        logAndWait("Driving to Net Position", netPosition);
        driveToPosition(netPosition);

        logAndWait("Scoring Preloaded Specimen", null);
        robot.setState(Robot.State.IDLE);
        robot.setState(Robot.State.SCORING);
        waitForRobotState("COMPLETED");

        Pose2d[] samplePositions = FieldConstants.getSamplePositions(teamColor);

        for (Pose2d samplePosition : samplePositions) {
            logAndWait("Driving to Sample Position", samplePosition);
            driveToPosition(samplePosition);

            logAndWait("Picking Up Sample", null);
            robot.setState(Robot.State.PICKUP);
            waitForRobotState("WAIT_FOR_DRIVE_FORWARD");

            logAndWait("Driving Forward to Pick Up Sample", null);
            driveForward(4);

            logAndWait("Completing Pickup", null);
            robot.onDriveForwardComplete();
            waitForRobotState("COMPLETED");

            logAndWait("Returning to Net Position", netPosition);
            driveToPosition(netPosition);

            logAndWait("Scoring Sample", null);
            robot.setState(Robot.State.SCORING);
            waitForRobotState("COMPLETED");
        }

        Pose2d ascentZonePosition = FieldConstants.getAscentZonePosition(teamColor);
        logAndWait("Driving to Ascent Zone Position", ascentZonePosition);
        driveToPosition(ascentZonePosition);

        logAndWait("Starting Level One Ascent", null);
        robot.setState(Robot.State.LEVEL_ONE_ASCENT);
        waitForRobotState("FINAL");
    }

    private void executeRightStartingPosition() {
        telemetry.addLine("Right Starting Position Not Yet Implemented.");
        telemetry.update();
    }

    private void driveToPosition(Pose2d target) {
        telemetry.addLine("Building Trajectory to Position...");
        telemetry.addData("Target Position", target);
        telemetry.addData("Current Position", driveTrain.getPoseEstimate());
        telemetry.update();

        TrajectorySequence sequence = driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                .splineTo(target.vec(), target.getHeading())
                .build();

        driveTrain.followTrajectorySequence(sequence);

        telemetry.addLine("Trajectory Complete.");
        telemetry.addData("Final Position", driveTrain.getPoseEstimate());
        telemetry.update();
    }

    private void driveForward(double distance) {
        telemetry.addLine("Building Forward Trajectory...");
        telemetry.addData("Distance", distance);
        telemetry.addData("Current Position", driveTrain.getPoseEstimate());
        telemetry.update();

        TrajectorySequence forwardSequence = driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                .forward(distance)
                .build();

        driveTrain.followTrajectorySequence(forwardSequence);

        telemetry.addLine("Forward Trajectory Complete.");
        telemetry.addData("Final Position", driveTrain.getPoseEstimate());
        telemetry.update();
    }

    private void logAndWait(String message, Pose2d position) {
        telemetry.addLine(message);
        if (position != null) {
            telemetry.addData("Target Position", position);
        }
        telemetry.addData("Current Position", driveTrain.getPoseEstimate());
        telemetry.addLine("Press 'A' to Continue...");
        telemetry.update();

        while (!gamepad1.a && opModeIsActive()) {
            sleep(50); // Wait for button press
        }

        telemetry.addLine("Continuing...");
        telemetry.update();
    }

    private void waitForRobotState(String expectedState) {
        while (opModeIsActive() && !robot.getCurrentSubstate().equals(expectedState)) {
            telemetry.addLine("Waiting for Robot State...");
            telemetry.addData("Expected State", expectedState);
            telemetry.addData("Current State", robot.getCurrentSubstate());
            telemetry.update();
            robot.update();
        }
        telemetry.addLine("State Reached.");
        telemetry.update();
    }
}