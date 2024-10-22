// File: TeleOpMode.java
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "TeleOp Mode")
public class TeleOpMode extends LinearOpMode {

    private DriveTrain driveTrain;
    private Robot robot;

    // Variable to track dev mode
    private boolean devMode = false;

    @Override
    public void runOpMode() {
        // Initialize the drivetrain
        driveTrain = new DriveTrain(this);

        // Initialize the robot
        robot = new Robot(hardwareMap);

        // Variables to track button states for edge detection
        boolean previousRightTriggerPressed = false;
        boolean previousHomeButtonPressed = false;
        boolean previousClawButtonPressed = false;
        boolean previousHangingButtonPressed = false;
        boolean previousScoringButtonPressed = false;
        boolean previousSelectButtonPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            // Handle drivetrain control
            driveTrain.drive(this);

            // Handle select button press to toggle dev mode
            boolean selectButtonPressed = gamepad2.back; // Assuming back button is the select button
            if (selectButtonPressed && !previousSelectButtonPressed) {
                devMode = !devMode; // Toggle dev mode
            }
            previousSelectButtonPressed = selectButtonPressed;

            if (!devMode) {
                // Normal mode

                // Handle button presses

                // Home State Button (e.g., gamepad2.y)
                if (gamepad2.y && !previousHomeButtonPressed) {
                    robot.onHomeButtonPressed();
                }
                previousHomeButtonPressed = gamepad2.y;

                // Switch Variation Button (e.g., right trigger)
                if (gamepad2.right_trigger > 0.5 && !previousRightTriggerPressed) {
                    robot.onSwitchVariationButtonPressed();
                }
                previousRightTriggerPressed = gamepad2.right_trigger > 0.5;

                // Claw Toggle Button (e.g., gamepad2.b)
                if (gamepad2.b && !previousClawButtonPressed) {
                    robot.onClawToggleButtonPressed();
                }
                previousClawButtonPressed = gamepad2.b;

                // Hanging Button (e.g., gamepad2.a)
                if (gamepad2.a && !previousHangingButtonPressed) {
                    robot.onHangingButtonPressed();
                }
                previousHangingButtonPressed = gamepad2.a;

                // Scoring Basket Button (e.g., gamepad2.x)
                if (gamepad2.x && !previousScoringButtonPressed) {
                    robot.onScoringBasketButtonPressed();
                }
                previousScoringButtonPressed = gamepad2.x;

                // Update the robot state
                robot.update();

                // Telemetry for debugging
                telemetry.addData("Mode", "Normal Mode");
                telemetry.addData("Current State", robot.currentState);
                telemetry.addData("Hanging Step", robot.hangingState.getCurrentStep());
                telemetry.addData("Scoring Step", robot.scoringBasketState.getCurrentStep());
                telemetry.addData("Home Variation", robot.homeState.getCurrentVariation());
            } else {
                // Dev mode
                // Provide manual control over the subsystems

                // Manual control of ViperLift using gamepad2 left stick y
                double viperLiftPower = -gamepad2.left_stick_y; // Up is negative
                robot.viperLift.setManualPower(viperLiftPower);

                // Manual control of Arm using gamepad2 right stick y
                double armPower = -gamepad2.right_stick_y; // Up is negative
                robot.arm.setManualPower(armPower);

                // Manual control of Wrist using gamepad2 dpad up/down
                if (gamepad2.dpad_up) {
                    robot.wrist.setManualPosition(robot.wrist.getPosition() + 0.01);
                } else if (gamepad2.dpad_down) {
                    robot.wrist.setManualPosition(robot.wrist.getPosition() - 0.01);
                }

                // Manual control of Claw using gamepad2.a and gamepad2.b
                if (gamepad2.a && !previousClawButtonPressed) {
                    robot.claw.open();
                } else if (gamepad2.b && !previousClawButtonPressed) {
                    robot.claw.close();
                }
                previousClawButtonPressed = gamepad2.a || gamepad2.b;

                // Telemetry for dev mode
                telemetry.addData("Mode", "Dev Mode");
                telemetry.addData("ViperLift Position", robot.viperLift.getCurrentPosition());
                telemetry.addData("ViperLift Power", viperLiftPower);
                telemetry.addData("Arm Position", robot.arm.getCurrentAngle());
                telemetry.addData("Arm Power", armPower);
                telemetry.addData("Wrist Position", robot.wrist.getPosition());
                telemetry.addData("Claw Position", robot.claw.getPosition());
            }

            // Telemetry common to both modes
            // Add any other telemetry values you need
            telemetry.update();
        }
    }
}