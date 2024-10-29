// File: AutonomousMode.java
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Autonomous Mode")
public class AutonomousMode extends LinearOpMode {

    private DriveTrainRR driveTrain;
    private Robot robot;

    @Override
    public void runOpMode() {
        // Initialize the drivetrain with Roadrunner integration
        driveTrain = new DriveTrainRR(hardwareMap);

        // Initialize the robot with the drivetrain
        robot = new Robot(hardwareMap, driveTrain);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            // Autonomous actions go here
            // For now, this is a placeholder where the autonomous strategy will be implemented

            // Example:
            // robot.performAutonomousActions();

            // Keep updating the robot state if necessary
            robot.update();

            // Optional: Add telemetry data
            telemetry.addData("Status", "Autonomous mode running");
            telemetry.update();
        }
    }
}