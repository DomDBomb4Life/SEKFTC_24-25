package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "TeleOp Mode Right")
public class TeleOpModeRight extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(hardwareMap);
        robot = new Robot(hardwareMap);
        robot.arm.OverridePosition();
        robot.arm.moveToAngle(90);

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
            handleDevModeToggle();

            if (devMode) {
                handleDevModeControls();
            } else {
                handleNormalModeControls();
            }

            handleCommonControls();

            // Update telemetry
            telemetry.addData("Mode", devMode ? "Dev Mode" : "Normal Mode");
            telemetry.addData("State", robot.getCurrentStateName());
            telemetry.addData("Substate", robot.getCurrentSubstate());
            telemetry.update();
        }
    }

    private void handleDevModeToggle() {
        boolean selectButtonPressed = gamepad2.back;
        if (selectButtonPressed && !previousSelectButtonPressed) {
            devMode = !devMode;
            if (!devMode) {
                robot.setState(robot.idleState);
            }
        }
        previousSelectButtonPressed = selectButtonPressed;
    }

    private void handleNormalModeControls() {
        if (isButtonJustPressed(gamepad2.y, previousYButtonPressed)) {
            robot.onHomeButtonPressed();
        }
        previousYButtonPressed = gamepad2.y;

        if (isTriggerJustPressed(gamepad2.right_trigger, previousRightTriggerPressed)) {
            robot.onRightTriggerPressed();
        }
        previousRightTriggerPressed = gamepad2.right_trigger > 0.5;

        if (isTriggerJustPressed(gamepad2.left_trigger, previousLeftTriggerPressed)) {
            robot.onPrimaryButtonPressed();
        }
        previousLeftTriggerPressed = gamepad2.left_trigger > 0.5;

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

    private void handleDevModeControls() {
        double threshold = 0.05;

        // Scale factors for adjustments
        double armIncrementScale = 1.0;        // Degrees per input unit
        int liftIncrementScale = 50;          // Encoder counts per input unit
        double servoIncrementScale = 1.0;     // Degrees per input unit

        double liftInput = -gamepad2.left_stick_y;
        if (Math.abs(liftInput) > threshold && robot.viperLift.isCloseToTarget()) {
            int liftIncrement = (int) (liftInput * liftIncrementScale);
            robot.viperLift.adjustTargetPosition(liftIncrement);
        }

        double armInput = -gamepad2.right_stick_y;
        if (Math.abs(armInput) > threshold && robot.arm.isCloseToTarget()) {
            double armIncrement = armInput * armIncrementScale;
            robot.arm.adjustTargetAngle(armIncrement);
        }

        double wristInput = 0.0;
        if (gamepad2.dpad_up) {
            wristInput = servoIncrementScale;
        } else if (gamepad2.dpad_down) {
            wristInput = -servoIncrementScale;
        }
        if (wristInput != 0.0) {
            robot.wrist.adjustAngle(wristInput);
        }

        double clawInput = 0.0;
        if (gamepad2.dpad_right) {
            clawInput = servoIncrementScale;
        } else if (gamepad2.dpad_left) {
            clawInput = -servoIncrementScale;
        }
        if (clawInput != 0.0) {
            robot.claw.adjustAngle(clawInput);
        }
    }

    private void handleCommonControls() {
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = -gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        driveTrain.updateSpeed(gamepad1.left_trigger, gamepad1.right_trigger);
        driveTrain.drive(leftStickY, leftStickX, rightStickX);

        telemetry.addData("Mode", devMode ? "Dev Mode" : "Normal Mode");
        telemetry.addData("ViperLift Position", robot.viperLift.getCurrentPosition());
        telemetry.addData("ViperLift Target", robot.viperLift.getTargetPosition());
        telemetry.addData("Arm Angle", robot.arm.getCurrentAngle());
        telemetry.addData("Arm Target", robot.arm.getTargetAngle());
        telemetry.addData("Wrist Angle", robot.wrist.getAngle());
        telemetry.addData("Wrist Target", robot.wrist.getTargetAngle());
        telemetry.addData("Claw Position", robot.claw.getAngle());
        telemetry.addData("Claw Target", robot.claw.getTargetAngle());
    }

    private boolean isButtonJustPressed(boolean currentState, boolean previousState) {
        return currentState && !previousState;
    }

    private boolean isTriggerJustPressed(double triggerValue, boolean previousState) {
        boolean currentState = triggerValue > 0.5;
        return currentState && !previousState;
    }
}