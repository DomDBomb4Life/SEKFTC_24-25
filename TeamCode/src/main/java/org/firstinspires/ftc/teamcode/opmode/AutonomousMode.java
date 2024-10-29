// File: AutonomousMode.java
package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousTrajectoryPlanner;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Autonomous Mode")
public class AutonomousMode extends LinearOpMode {

    private AutonomousTrajectoryPlanner trajectoryPlanner;
    private Robot robot;

    @Override
    public void runOpMode() {
        // Initialize the trajectory planner and robot
        trajectoryPlanner = new AutonomousTrajectoryPlanner(hardwareMap);
        robot = new Robot(hardwareMap, trajectoryPlanner.getDriveTrain());

        // Initialize trajectory planner
        trajectoryPlanner.initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            // Update localization
            trajectoryPlanner.updateLocalization();

            // Execute the alliance sample trajectory
            telemetry.addData("Phase", "Collecting Alliance Samples");
            telemetry.update();
            trajectoryPlanner.executeTrajectorySequence(trajectoryPlanner.buildAllianceSampleTrajectory());

            // Update localization
            trajectoryPlanner.updateLocalization();

            // Execute the yellow sample scoring trajectory
            telemetry.addData("Phase", "Scoring Yellow Samples");
            telemetry.update();
            trajectoryPlanner.executeTrajectorySequence(trajectoryPlanner.buildYellowSampleScoringTrajectory());

            // Update localization
            trajectoryPlanner.updateLocalization();

            // Execute the level one ascent trajectory
            telemetry.addData("Phase", "Level One Ascent");
            telemetry.update();
            trajectoryPlanner.executeTrajectorySequence(trajectoryPlanner.buildLevelOneAscentTrajectory());

            // Keep updating the robot state if necessary
            robot.update();

            // Final telemetry update
            telemetry.addData("Status", "Autonomous mode complete");
            telemetry.update();

            // Shutdown resources
            trajectoryPlanner.shutdown();
        }
    }
}