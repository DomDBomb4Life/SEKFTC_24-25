// File: AutonomousOpMode.java
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.robot.Robot;
// Removed direct import of LevelOneAscentState

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Autonomous Op Mode that allows pre-match configuration via Gamepad 1.
 */
@Autonomous(name = "Autonomous Op Mode")
public class AutonomousOpMode extends LinearOpMode {

    // Variables to store team color and starting position
    private enum TeamColor {
        RED,
        BLUE
    }

    private enum StartingPosition {
        LEFT,
        RIGHT
    }

    private TeamColor teamColor = TeamColor.BLUE;
    private StartingPosition startingPosition = StartingPosition.LEFT;

    // Button press trackers
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;

    // Robot and drivetrain instances
    private Robot robot;
    private DriveTrainRR driveTrain;

    @Override
    public void runOpMode() {
        // Initialize robot subsystems
        robot = new Robot(hardwareMap);
        driveTrain = new DriveTrainRR(hardwareMap);

        // Initialize starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        driveTrain.setPoseEstimate(startPose);

        onInit();


        // Pre-match configuration loop
        while (!isStarted() && !isStopRequested()) {
            handleMenuSelection();

            // Display current selections
            telemetry.addData("Team Color", teamColor);
            telemetry.addData("Starting Position", startingPosition);
            telemetry.update();

            sleep(100); // Prevent button spamming
        }

        // Wait for start
        waitForStart();

        if (isStopRequested()) return;

        // Execute autonomous actions based on selections
        if (startingPosition == StartingPosition.RIGHT) {
            executeRightStartingPosition();
        } else {
            executeLeftStartingPosition();
        }
        while (opModeIsActive()){
            robot.update();
        }

    }

    // Method to handle menu selection using Gamepad 1
    private void handleMenuSelection() {
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (dpadUp && !previousDpadUp) {
            // Toggle team color
            teamColor = (teamColor == TeamColor.RED) ? TeamColor.BLUE : TeamColor.RED;
        }

        if (dpadLeft && !previousDpadLeft) {
            // Toggle starting position
            startingPosition = (startingPosition == StartingPosition.LEFT) ? StartingPosition.RIGHT : StartingPosition.LEFT;
        }

        // Update previous button states
        previousDpadUp = dpadUp;
        previousDpadDown = dpadDown;
        previousDpadLeft = dpadLeft;
        previousDpadRight = dpadRight;
    }

    // Autonomous routine for the right starting position
    private void executeRightStartingPosition() {
        double strafeDirection = 1.0;
        // Robot is assumed to be turned 90 degrees towards the field
        TrajectorySequence sequence = driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                .forward(2.4)
                .strafeLeft(strafeDirection * 24*2) // Strafe left or right by 1 tile
                // Drive forward 24 inches (adjust as needed)
                .build();

        driveTrain.followTrajectorySequence(sequence);
    }

    // Autonomous routine for the left starting position
    private void executeLeftStartingPosition() {
        // Calculate adjusted trajectories based on team color
        double strafeDirection = -1.0;

        TrajectorySequence sequence = driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                .forward(2.4) // Drive forward 0.1 tiles (1 tile = 24 inches)
                .strafeLeft(strafeDirection * 24) // Strafe left or right by 1 tile
                .forward(60) // Drive forward 2.5 tiles (60 inches)
                .turn(Math.toRadians(-90)) // Turn 90 degrees right
                .build();

        driveTrain.followTrajectorySequence(sequence);

        // Start the Level One Ascent State through the robot class
        robot.onLevelOneAscentButtonPressed();

        // Update robot until it reaches WAIT_FOR_TRIGGER step
        while (opModeIsActive() && !robot.getCurrentSubstate().equals("WAIT_FOR_TRIGGER")) {
            robot.update();
            driveTrain.update();
        }

        // Drive forward by 0.25 tiles (6 inches)
        TrajectorySequence forwardSequence = driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                .forward(6) // 0.25 tiles = 6 inches
                .build();

        driveTrain.followTrajectorySequence(forwardSequence);

        // Activate the second half of the Level One Ascent State
        robot.onRightTriggerPressed();

        // Update robot until it reaches FINAL step
        while (opModeIsActive() && !robot.getCurrentSubstate().equals("FINAL")) {
            robot.update();
            driveTrain.update();
        }
    }
    private void onInit(){
        robot.wrist.setAngle(20);
        robot.arm.moveToAngle(137.0);
    }
}