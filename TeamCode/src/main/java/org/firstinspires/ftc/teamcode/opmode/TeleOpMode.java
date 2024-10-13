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

        waitForStart();

        while (opModeIsActive()) {
            // Handle drivetrain control
            driveTrain.drive(this);

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
            telemetry.addData("Current State", robot.currentState);
            telemetry.addData("Hanging Step", robot.hangingState.getCurrentStep());
            telemetry.addData("Scoring Step", robot.scoringBasketState.getCurrentStep());
            telemetry.addData("Home Variation", robot.homeState.getCurrentVariation());
            telemetry.update();
        }
    }
}