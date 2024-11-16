// File: BackAndForth.java
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;

/**
 * Op mode for preliminary tuning of the follower PID coefficients.
 */
@Config
@TeleOp(group = "drive")
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 50; // in

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrainRR drive = new DriveTrainRR(hardwareMap);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .back(DISTANCE)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(new Pose2d())
                            .forward(DISTANCE)
                            .back(DISTANCE)
                            .build()
            );
        }
    }
}