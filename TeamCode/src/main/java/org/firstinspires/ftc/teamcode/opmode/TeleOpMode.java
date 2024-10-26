// File: TeleOpMode.java
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.Robot;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@TeleOp(name = "TeleOp Mode")
public class TeleOpMode extends LinearOpMode {

    private DriveTrain driveTrain;
    private Robot robot;

    // Variable to track dev mode
    private boolean devMode = false;

    // Variable to track aimbot activation button state
    private boolean previousAimbotButtonPressed = false;

    // Button toggle states
    private boolean previousSelectButtonPressed = false;

    @Override
    public void runOpMode() {
        // Initialize the drivetrain
        driveTrain = new DriveTrain(hardwareMap);

        // Initialize the robot with the drivetrain
        robot = new Robot(hardwareMap, driveTrain);

        waitForStart();

        while (opModeIsActive()) {
            // Update robot systems
            robot.update();

            // Handle dev mode toggle
            handleDevModeToggle();

            // Handle aimbot activation
            handleAimbot();

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

    // Method to handle aimbot activation
    private void handleAimbot() {
        boolean aimbotButtonPressed = gamepad1.a;
        if (aimbotButtonPressed && !previousAimbotButtonPressed) {
            if (robot.aimbotController.isActive()) {
                robot.deactivateAimbot();
            } else {
                // Set target pose, could be based on vision data
                Pose2d targetPose = robot.localization.getCurrentPose(); // Placeholder, replace with actual target
                robot.activateAimbot(targetPose);
            }
        }
        previousAimbotButtonPressed = aimbotButtonPressed;

        // If aimbot is active, skip manual driving
        if (!robot.aimbotController.isActive()) {
            // Read gamepad inputs
            double leftStickY = -gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = -gamepad1.right_stick_x;

            // Update speed based on triggers
            driveTrain.updateSpeed(gamepad1.left_trigger, gamepad1.right_trigger);

            // Handle drivetrain control
            driveTrain.drive(leftStickY, leftStickX, rightStickX);
        }
    }

    // Method to handle normal mode controls
    private void handleNormalModeControls() {
        // Handle button presses with helper methods
        if (isButtonJustPressed(gamepad2.y)) {
            robot.onHomeButtonPressed();
        }

        if (isTriggerJustPressed(gamepad2.right_trigger)) {
            robot.onSwitchVariationButtonPressed();
        }

        if (isButtonJustPressed(gamepad2.b)) {
            robot.onClawToggleButtonPressed();
        }

        if (isButtonJustPressed(gamepad2.a)) {
            robot.onHangingButtonPressed();
        }

        if (isButtonJustPressed(gamepad2.x)) {
            robot.onScoringButtonPressed();
        }

        // Telemetry for debugging
        telemetry.addData("Mode", "Normal Mode");
        telemetry.addData("Current State", robot.currentState);
        telemetry.addData("Speed Multiplier", driveTrain.getSpeedMultiplier());
        telemetry.addData("Aimbot Active", robot.aimbotController.isActive());
    }

    // Method to handle dev mode controls
    private void handleDevModeControls() {
        // Threshold to prevent unintentional adjustments
        double threshold = 0.05;

        // Scale factors for adjustments
        double armIncrementScale = 1.0;       // Degrees per input unit
        int liftIncrementScale = 50;          // Encoder counts per input unit
        double servoIncrementScale = 0.01;    // Servo increment per input unit

        // Manual control of ViperLift
        double liftInput = -gamepad2.left_stick_y; // Up is negative
        if (Math.abs(liftInput) > threshold && !robot.viperLift.isBusy()) {
            int liftIncrement = (int) (liftInput * liftIncrementScale);
            robot.viperLift.adjustTargetPosition(liftIncrement);
        }

        // Manual control of Arm
        double armInput = -gamepad2.right_stick_y; // Up is negative
        if (Math.abs(armInput) > threshold && !robot.arm.isBusy()) {
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
            robot.wrist.setManualPosition(robot.wrist.getPosition() + wristInput);
        }

        // Manual control of Claw
        double clawInput = 0.0;
        if (gamepad2.dpad_right) {
            clawInput = servoIncrementScale;
        } else if (gamepad2.dpad_left) {
            clawInput = -servoIncrementScale;
        }
        if (clawInput != 0.0) {
            robot.claw.setManualPosition(robot.claw.getPosition() + clawInput);
        }

        // Telemetry for dev mode
        telemetry.addData("Mode", "Dev Mode");
        telemetry.addData("ViperLift Position", robot.viperLift.getCurrentPosition());
        telemetry.addData("ViperLift Target", robot.viperLift.getTargetPosition());
        telemetry.addData("Arm Angle", robot.arm.getCurrentAngle());
        telemetry.addData("Arm Target", robot.arm.getTargetAngle());
        telemetry.addData("Wrist Position", robot.wrist.getPosition());
        telemetry.addData("Claw Position", robot.claw.getPosition());
    }

    // Helper method for detecting button press edges
    private boolean isButtonJustPressed(boolean currentState) {
        return currentState && !gamepad2.start;
    }

    // Helper method for detecting trigger press edges
    private boolean isTriggerJustPressed(double triggerValue) {
        return triggerValue > 0.5 && !gamepad2.start;
    }
}