// File: TeleOpMode.java
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.states.ScoringSpecimenState;

@TeleOp(name = "TeleOp Mode")
public class TeleOpMode extends LinearOpMode {

    private DriveTrain driveTrain;
    private DriveTrainRR driveTrainRR;

    private Robot robot;

    // Variable to track dev mode
    private boolean devMode = false;

    // Button toggle states
    private boolean previousSelectButtonPressed = false;
    private boolean previousYButtonPressed = false;
    private boolean previousAButtonPressed = false;
    private boolean previousBButtonPressed = false;
    private boolean previousXButtonPressed = false;
    private boolean previousRightTriggerPressed = false;
    private boolean previousDpadUpPressed = false;
    private boolean previousDpadDownPressed = false;
    private boolean previousDpadRightPressed = false;

    @Override
    public void runOpMode() {
        // Initialize the drivetrain
        driveTrainRR = new DriveTrainRR(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);

        // Initialize the robot with the drivetrain
        robot = new Robot(hardwareMap, driveTrainRR);

        waitForStart();

        while (opModeIsActive()) {
            // Update robot systems
            robot.update();

            // Handle dev mode toggle
            handleDevModeToggle();

            if (devMode) {
                handleDevModeControls();
            } else {
                handleNormalModeControls();
            }

            // Update telemetry
            telemetry.update();
        }
    }

    // Method to handle dev mode toggle
    private void handleDevModeToggle() {
        boolean selectButtonPressed = gamepad2.back;
        if (selectButtonPressed && !previousSelectButtonPressed) {
            devMode = !devMode; // Toggle dev mode
            if (devMode) {
                telemetry.addData("Mode", "Switched to Dev Mode");
            } else {
                telemetry.addData("Mode", "Switched to Normal Mode");
                // Optionally reset to idle state when exiting dev mode
                robot.setState(Robot.State.IDLE);
            }
        }
        previousSelectButtonPressed = selectButtonPressed;
    }

    // Method to handle normal mode controls
    private void handleNormalModeControls() {
        // Handle button presses with helper methods
        if (isButtonJustPressed(gamepad2.y, previousYButtonPressed)) {
            robot.onHomeButtonPressed();
        }
        previousYButtonPressed = gamepad2.y;

        if (isTriggerJustPressed(gamepad2.right_trigger, previousRightTriggerPressed)) {
            robot.onSwitchVariationButtonPressed();
        }
        previousRightTriggerPressed = gamepad2.right_trigger > 0.5;

        if (isButtonJustPressed(gamepad2.b, previousBButtonPressed)) {
            robot.onClawToggleButtonPressed();
        }
        previousBButtonPressed = gamepad2.b;

        if (isButtonJustPressed(gamepad2.a, previousAButtonPressed)) {
            robot.onHangingButtonPressed();
        }
        previousAButtonPressed = gamepad2.a;

        if (isButtonJustPressed(gamepad2.x, previousXButtonPressed)) {
            robot.onScoringButtonPressed();
        }
        previousXButtonPressed = gamepad2.x;

        // Handle new states
        if (isButtonJustPressed(gamepad2.dpad_up, previousDpadUpPressed)) {
            robot.onObservationButtonPressed();
        }
        previousDpadUpPressed = gamepad2.dpad_up;

        if (isButtonJustPressed(gamepad2.dpad_down, previousDpadDownPressed)) {
            robot.onScoringSpecimenButtonPressed();
        }
        previousDpadDownPressed = gamepad2.dpad_down;

        // Read gamepad inputs for driving
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = -gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        // Update speed based on triggers
        driveTrain.updateSpeed(gamepad1.left_trigger, gamepad1.right_trigger);

        // Handle drivetrain control
        driveTrain.drive(leftStickY, leftStickX, rightStickX);

        // Handle updates for ScoringSpecimenState
        if (robot.currentState == Robot.State.SCORING_SPECIMEN) {
            robot.scoringSpecimenState.update(
                gamepad2.dpad_down, // Primary button
                gamepad2.dpad_right, // Secondary button
                previousDpadDownPressed,
                previousDpadRightPressed
            );
            previousDpadDownPressed = gamepad2.dpad_down;
            previousDpadRightPressed = gamepad2.dpad_right;
        }

        // Telemetry for normal mode
        telemetry.addData("Mode", devMode ? "Dev Mode" : "Normal Mode");
        telemetry.addData("Current State", robot.currentState);
        telemetry.addData("Substate", robot.getCurrentSubstate());
        telemetry.addData("ViperLift Position", robot.viperLift.getCurrentPosition());
        telemetry.addData("ViperLift Target", robot.viperLift.getTargetPosition());
        telemetry.addData("Arm Angle", robot.arm.getCurrentAngle());
        telemetry.addData("Arm Target", robot.arm.getTargetAngle());
        telemetry.addData("Wrist Position", robot.wrist.getPosition());
        telemetry.addData("Wrist Target", robot.wrist.getTargetPosition());
        telemetry.addData("Claw Position", robot.claw.getPosition());
        telemetry.addData("Claw Target", robot.claw.getTargetPosition());
    }

// In handleDevModeControls()
private void handleDevModeControls() {
    // Threshold to prevent unintentional adjustments
    double threshold = 0.05;

    // Scale factors for adjustments
    double armIncrementScale = 1.0;       // Degrees per input unit
    int liftIncrementScale = 50;          // Encoder counts per input unit
    double servoIncrementScale = 1.0;    // Degrees per input unit

    // Manual control of ViperLift
    double liftInput = -gamepad2.left_stick_y; // Up is negative
    if (Math.abs(liftInput) > threshold && robot.viperLift.isCloseToTarget()) {
        int liftIncrement = (int) (liftInput * liftIncrementScale);
        robot.viperLift.adjustTargetPosition(liftIncrement);
    }

    // Manual control of Arm
    double armInput = -gamepad2.right_stick_y; // Up is negative
    if (Math.abs(armInput) > threshold && robot.arm.isCloseToTarget()) {
        double armIncrement = armInput * armIncrementScale;
        robot.arm.adjustTargetAngle(armIncrement);
    }

    // Manual control of Wrist
    double wristInput = 0.0;
    if (gamepad2.dpad_up) {
        wristInput = servoIncrementScale;
    } else if (gamepad2.dpad_down) {
        wristInput = -servoIncrementScale;
    }
    if (wristInput != 0.0) {
        robot.wrist.adjustAngle(wristInput);
    }

    // Manual control of Claw
    double clawInput = 0.0;
    if (gamepad2.dpad_right) {
        clawInput = servoIncrementScale;
    } else if (gamepad2.dpad_left) {
        clawInput = -servoIncrementScale;
    }
    if (clawInput != 0.0) {
        robot.claw.adjustAngle(clawInput);
    }

    // Telemetry for dev mode
    telemetry.addData("Mode", "Dev Mode");
    telemetry.addData("ViperLift Position", robot.viperLift.getCurrentPosition());
    telemetry.addData("ViperLift Target", robot.viperLift.getTargetPosition());
    telemetry.addData("Arm Angle", robot.arm.getCurrentAngle());
    telemetry.addData("Arm Target", robot.arm.getTargetAngle());
    telemetry.addData("Wrist Angle", robot.wrist.getAngle());
    telemetry.addData("Wrist Target", robot.wrist.getTargetAngle());
    telemetry.addData("Claw Angle", robot.claw.getAngle());
    telemetry.addData("Claw Target", robot.claw.getTargetAngle());
}

    // Helper method for detecting button press edges
    private boolean isButtonJustPressed(boolean currentState, boolean previousState) {
        return currentState && !previousState;
    }

    // Helper method for detecting trigger press edges
    private boolean isTriggerJustPressed(double triggerValue, boolean previousState) {
        boolean currentState = triggerValue > 0.5;
        return currentState && !previousState;
    }
}