// File: AutonomousOpMode.java
package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousTrajectoryPlanner;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "AutonomousOpMode", group = "Competition")
public class AutonomousOpMode extends LinearOpMode {

    private Robot robot;
    private DriveTrainRR driveTrainRR;
    private AutonomousTrajectoryPlanner trajectoryPlanner;
    private Action autonomousSequence;

    @Override
    public void runOpMode() {
        // Initialize hardware
        driveTrainRR = new DriveTrainRR(hardwareMap);
        robot = new Robot(hardwareMap, driveTrainRR);
        trajectoryPlanner = new AutonomousTrajectoryPlanner(hardwareMap);

        // Initialize the trajectory planner
        trajectoryPlanner.initialize();

        // Build the autonomous sequence
        autonomousSequence = trajectoryPlanner.buildAutonomousSequence();

        // Telemetry for initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start
        waitForStart();

        if (opModeIsActive()) {
            // Start the autonomous sequence
            trajectoryPlanner.executeAutonomousSequence(autonomousSequence);

            // Loop while op mode is active and robot is running
            while (opModeIsActive()) {
                // Update robot systems
                robot.update();

                // Update trajectory planner localization
                trajectoryPlanner.updateLocalization();

                // Telemetry data
                telemetry.addData("Pose X", robot.getDriveTrain().getPoseEstimate().position.x);
                telemetry.addData("Pose Y", robot.getDriveTrain().getPoseEstimate().position.y);
//                telemetry.addData("Heading (deg)", Math.toDegrees(robot.getDriveTrain().getPoseEstimate().heading.getRadians()));
                telemetry.update();

                idle();
            }
        }

        // Shutdown procedures
        trajectoryPlanner.shutdown();
    }
}