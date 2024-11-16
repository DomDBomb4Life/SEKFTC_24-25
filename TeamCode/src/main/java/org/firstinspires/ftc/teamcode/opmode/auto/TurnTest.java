// File: TurnTest.java
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;

/**
 * Simple op mode to test turning capabilities.
 */
@Config
@TeleOp(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // degrees

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrainRR drive = new DriveTrainRR(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}