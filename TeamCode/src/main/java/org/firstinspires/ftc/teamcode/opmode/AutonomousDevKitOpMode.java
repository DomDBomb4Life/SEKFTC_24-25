// File: AutonomousDevKitOpMode.java
package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousTrajectoryPlanner;
import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.field.FieldMap;
import org.firstinspires.ftc.teamcode.vision.VisionSystem;

/**
 * Autonomous Development Kit OpMode for testing and calibration.
 * Provides tools to test trajectories, vision localization, and allows manual control.
 */
@TeleOp(name = "Autonomous DevKit OpMode")
public class AutonomousDevKitOpMode extends LinearOpMode {

    private AutonomousTrajectoryPlanner trajectoryPlanner;
    private VisionLocalization visionLocalization;
    private DriveTrainRR driveTrainRR;
    private DriveTrain driveTrain; // For manual control
    private Robot robot; // Robot subsystems

    private boolean isRunningAutonomous = false;
    private Thread autonomousThread;

    @Override
    public void runOpMode() {
        // Initialize subsystems
        trajectoryPlanner = new AutonomousTrajectoryPlanner(hardwareMap);
        visionLocalization = new VisionLocalization(trajectoryPlanner.getVisionSystem());
        driveTrainRR = trajectoryPlanner.getDriveTrain();
        driveTrain = new DriveTrain(hardwareMap); // For manual control
        robot = new Robot(hardwareMap, driveTrainRR); // Initialize robot with DriveTrainRR

        // Initialize the trajectory planner
        trajectoryPlanner.initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean aButtonPrev = false;
        boolean bButtonPrev = false;
        boolean xButtonPrev = false;
        boolean yButtonPrev = false;

        while (opModeIsActive()) {
            // Update localization
            trajectoryPlanner.updateLocalization();

            // Retrieve and display the fused pose
            Pose2d currentPose = driveTrainRR.getPoseEstimate();
            telemetry.addData("Current Pose", currentPose);

            // Display vision pose
            Pose2d visionPose = visionLocalization.getVisionPose();
            telemetry.addData("Vision Pose", visionPose != null ? visionPose : "No detections");

            // Handle manual driving if not running autonomous
            if (!isRunningAutonomous) {
                handleManualDriving();
            }

            // Button A: Run Autonomous Sequence
            if (gamepad1.a && !aButtonPrev) {
                if (!isRunningAutonomous) {
                    isRunningAutonomous = true;
                    telemetry.addData("Action", "Starting Autonomous Sequence");
                    autonomousThread = new Thread(() -> {
                        Action sequence = trajectoryPlanner.buildAutonomousSequence();
                        trajectoryPlanner.executeAutonomousSequence(sequence);
                        isRunningAutonomous = false;
                    });
                    autonomousThread.start();
                } else {
                    telemetry.addData("Action", "Autonomous already running");
                }
            }
            aButtonPrev = gamepad1.a;

            // Button B: Interrupt Autonomous
            if (gamepad1.b && !bButtonPrev) {
                if (isRunningAutonomous) {
                    autonomousThread.interrupt();
                    isRunningAutonomous = false;
                    telemetry.addData("Action", "Autonomous Interrupted");
                }
            }
            bButtonPrev = gamepad1.b;

            // Button X: Calibrate Vision Localization
            if (gamepad1.x && !xButtonPrev) {
                // Implement calibration routine
                calibrateVisionLocalization();
                telemetry.addData("Action", "Calibrated Vision Localization");
            }
            xButtonPrev = gamepad1.x;

            // Button Y: Reset Pose Estimate
            if (gamepad1.y && !yButtonPrev) {
                driveTrainRR.setPoseEstimate(FieldMap.STARTING_POSITION);
                telemetry.addData("Action", "Reset Pose Estimate");
            }
            yButtonPrev = gamepad1.y;

            telemetry.update();
        }

        // Shutdown systems
        if (isRunningAutonomous && autonomousThread != null) {
            autonomousThread.interrupt();
        }
        trajectoryPlanner.shutdown();
    }

    /**
     * Handles manual driving using gamepad controls.
     */
    private void handleManualDriving() {
        // Read gamepad inputs for driving
        double leftStickY = -gamepad1.left_stick_y;
        double leftStickX = -gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        // Update speed based on triggers
        driveTrain.updateSpeed(gamepad1.left_trigger, gamepad1.right_trigger);

        // Handle drivetrain control
        driveTrain.drive(leftStickY, leftStickX, rightStickX);
    }

    /**
     * Calibrates the vision localization system.
     */
    private void calibrateVisionLocalization() {
        // Implement calibration logic here
        // For example, adjust camera parameters or tag layouts
    }
}