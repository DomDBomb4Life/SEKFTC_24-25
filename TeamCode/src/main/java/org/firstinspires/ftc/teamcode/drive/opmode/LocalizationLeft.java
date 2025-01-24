package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.util.DashboardUtil.drawPoseHistory;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;





import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;

import java.util.LinkedList;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationLeft extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    Canvas fieldOverlay = packet.fieldOverlay();
    private FtcDashboard dashboard;
    private  LinkedList<Pose2d> poseHistory = new LinkedList<>();
    public static int POSE_HISTORY_LIMIT = 100;



    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrainRR drive = new DriveTrainRR(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speed;
        Pose2d startPose = FieldConstants.LEFT_START;
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);


        drive.setPoseEstimate(startPose);
        waitForStart();
        if (gamepad1.right_bumper) {
            speed = 0.8;
        } else {
            speed = 0.5;
        }

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    ).times(speed)
            );

            drive.update();


            Pose2d poseEstimate = drive.getPoseEstimate();
            poseHistory.add(poseEstimate);
            if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
                poseHistory.removeFirst();
            }
            drawPoseHistory(fieldOverlay, poseHistory);



            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
//            dashboard.sendTelemetryPacket(packet);


        }
    }
}
