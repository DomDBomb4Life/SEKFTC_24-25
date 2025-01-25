package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "TeleOp Mode")
public class TeleOpMode extends LinearOpMode {

    private DriveTrain driveTrain;
    private Robot robot;

    private boolean devMode = false;

    // Button toggle states
    private boolean previousSelectButtonPressed = false;
    private boolean previousYButtonPressed = false;
    private boolean previousAButtonPressed = false;
    private boolean previousBButtonPressed = false;
    private boolean previousXButtonPressed = false;
    private boolean previousLeftBumperPressed = false;
    private boolean previousRightTriggerPressed = false;
    private boolean previousLeftTriggerPressed = false;

    private boolean previousDpadUpPressed = false;
    private boolean previousDpadDownPressed = false;

    // Dev mode winch buttons on gamepad2
    private boolean previousYWinch = false;
    private boolean previousXWinch = false;
    private boolean previousBWinch = false;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        robot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // Update all subsystems
            robot.update();

            // Toggle dev mode if 'back' on gamepad2
            handleDevModeToggle();

            // If dev mode is active, use dev controls; otherwise normal controls
            if (devMode) {
                handleDevModeControls();
            } else {
                handleNormalModeControls();
            }

            // Common logic for drive, telemetry, and manual winch spool
            handleCommonControls();

            // Show basic telemetry
            telemetry.addData("Mode", devMode ? "Dev Mode" : "Normal Mode");
            telemetry.addData("State", robot.getCurrentStateName());
            telemetry.addData("Substate", robot.getCurrentSubstate());
            telemetry.update();
        }
    }

    /**
     * Toggles devMode when the 'back' button on gamepad2 is pressed.
     * If devMode is turned off, reset the robot to idle.
     */
    private void handleDevModeToggle() {
        boolean selectButtonPressed = gamepad2.back;
        if (selectButtonPressed && !previousSelectButtonPressed) {
            devMode = !devMode;
            if (!devMode) {
                // Return to idle state if exiting dev mode
                robot.setState(robot.idleState);
            }
        }
        previousSelectButtonPressed = selectButtonPressed;
    }

    /**
     * Normal TeleOp controls (the original logic when devMode == false).
     */
    private void handleNormalModeControls() {
        robot.winch.runToPosition =true;
        if (isButtonJustPressed(gamepad2.y, previousYButtonPressed)) {
            robot.onHomeButtonPressed();
        }
        previousYButtonPressed = gamepad2.y;

        if (isTriggerJustPressed(gamepad2.right_trigger, previousRightTriggerPressed)) {
            robot.onRightTriggerPressed();
        }
        previousRightTriggerPressed = (gamepad2.right_trigger > 0.5);

        if (isTriggerJustPressed(gamepad2.left_trigger, previousLeftTriggerPressed)) {
            robot.onPrimaryButtonPressed();
        }
        previousLeftTriggerPressed = (gamepad2.left_trigger > 0.5);

        if (isButtonJustPressed(gamepad2.b, previousBButtonPressed)) {
            robot.onClawToggleButtonPressed();
        }
        previousBButtonPressed = gamepad2.b;

        if (isButtonJustPressed(gamepad2.a, previousAButtonPressed)) {
            robot.onLevelOneAscentButtonPressed();
        }
        previousAButtonPressed = gamepad2.a;

        if (isButtonJustPressed(gamepad2.x, previousXButtonPressed)) {
            robot.onScoringButtonPressed();
        }
        previousXButtonPressed = gamepad2.x;

        if (isButtonJustPressed(gamepad2.dpad_down, previousDpadDownPressed)) {
            robot.onObservationButtonPressed();
        }
        previousDpadDownPressed = gamepad2.dpad_down;

        if (isButtonJustPressed(gamepad2.dpad_up, previousDpadUpPressed)) {
            robot.onScoringSpecimenButtonPressed();
        }
        previousDpadUpPressed = gamepad2.dpad_up;

        if (isButtonJustPressed(gamepad2.left_bumper, previousLeftBumperPressed)) {
            robot.onInitArmButtonPresses();
        }
        previousLeftBumperPressed = gamepad2.left_bumper;
    }

    /**
     * Dev mode controls (when devMode == true).
     * Lets us fine-tune the arm, lift, wrist, claw, plus control the winch mode (ATTACHED, DETACHED, OVERRIDE).
     */
    private void handleDevModeControls() {
        double threshold = 0.05;

        // Scale factors for small incremental adjustments
        double armIncrementScale = 1.0;    // Degrees per input unit
        int liftIncrementScale = 100;      // Encoder counts per input unit
        double servoIncrementScale = 1.0; // Degrees per input unit
        int winchIncrementScale = 50;


        // ------ ViperLift Manual Increments ------
        double liftInput = -gamepad2.left_stick_y;
        if (Math.abs(liftInput) > threshold && robot.viperLift.isCloseToTarget()) {
            int liftIncrement = (int) (liftInput * liftIncrementScale);
            robot.viperLift.adjustTargetPosition(liftIncrement);
        }

        // ------ Arm Manual Increments ------
        double armInput = -gamepad2.right_stick_y;
        if (Math.abs(armInput) > threshold && robot.arm.isCloseToTarget()) {
            double armIncrement = armInput * armIncrementScale;
            robot.arm.adjustTargetAngle(armIncrement);
        }

        // ------ Wrist Manual Increments via DPad Up/Down ------
        double wristInput = 0.0;
        if (gamepad2.dpad_up) {
            wristInput = servoIncrementScale;
        } else if (gamepad2.dpad_down) {
            wristInput = -servoIncrementScale;
        }
        if (wristInput != 0.0) {
            robot.wrist.adjustAngle(wristInput);
        }

        // ------ Claw Manual Increments via DPad Left/Right ------
        double clawInput = 0.0;
        if (gamepad2.dpad_right) {
            clawInput = servoIncrementScale;
        } else if (gamepad2.dpad_left) {
            clawInput = -servoIncrementScale;
        }
        if (clawInput != 0.0) {
            robot.claw.adjustAngle(clawInput);
        }

        //         ------ Manual Winch Spool (gamepad2 bumpers) ------
        // right_bumper => spool up, left_bumper => spool down
        double spoolPower = 0.0;
        if (gamepad2.right_bumper) {
            robot.winch.adjustTargetAngle(winchIncrementScale);
        } else if (gamepad2.left_bumper) {
            robot.winch.adjustTargetAngle(-winchIncrementScale);
        }

        if (isButtonJustPressed(gamepad2.a, previousAButtonPressed)) {
        }
        previousAButtonPressed = gamepad2.a;
        // This sets the winch to OVERRIDE mode with the given power


    }

    /**
     * Common controls for both Normal and Dev modes:
     *  - Driving with gamepad1
     *  - Telemetry for positions and angles
     *  - Winch spool up/down (gamepad2 bumpers)
     */
    private void handleCommonControls() {
        // ------ Drive ------
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = -gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;


        driveTrain.updateSpeed(gamepad1.left_trigger, gamepad1.right_trigger);
        driveTrain.drive(leftStickY, leftStickX, rightStickX);
//


        // ------ Telemetry ------
        telemetry.addData("Mode", devMode ? "Dev Mode" : "Normal Mode");
        telemetry.addData("ViperLift Position", robot.viperLift.getCurrentPosition());
        telemetry.addData("ViperLift Target", robot.viperLift.getTargetPosition());
        telemetry.addData("Arm Angle", robot.arm.getCurrentAngle());
        telemetry.addData("Arm Target", robot.arm.getTargetAngle());
        telemetry.addData("Wrist Angle", robot.wrist.getAngle());
        telemetry.addData("Wrist Target", robot.wrist.getTargetAngle());
        telemetry.addData("Claw Position", robot.claw.getAngle());
        telemetry.addData("Claw Target", robot.claw.getTargetAngle());
        telemetry.addData("Claw Servo Position", robot.claw.getPosition());

        // Show Winch info
        telemetry.addData("Winch Mode", robot.winch.getMode());
        telemetry.addData("Winch Current Pos", robot.winch.getCurrentPosition());
        telemetry.addData("Winch Target Pos", robot.winch.getTargetPosition());
        telemetry.addData("Winch runToPosition?", robot.winch.isRunToPosition());
    }

    // ---------- Helper for detecting a rising edge on buttons ----------
    private boolean isButtonJustPressed(boolean currentState, boolean previousState) {
        return currentState && !previousState;
    }

    // ---------- Helper for triggers as a button press if above 0.5 ----------
    private boolean isTriggerJustPressed(double triggerValue, boolean previousState) {
        boolean currentState = (triggerValue > 0.5);
        return currentState && !previousState;
    }
}