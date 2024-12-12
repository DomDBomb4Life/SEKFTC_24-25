//package org.firstinspires.ftc.teamcode.opmode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.*;
//import com.qualcomm.robotcore.eventloop.opmode.*;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
//import org.firstinspires.ftc.teamcode.trajectorysequence1.TrajectorySequence;
//
///**
// * Combined OpMode that allows testing different RoadRunner functionalities based on gamepad input.
// */
//@Config
//@TeleOp(name = "Combined Test OpMode", group = "drive")
//public class CombinedTestOpMode extends LinearOpMode {
//
//    public static double DISTANCE = 60; // inches
//    public static double ANGLE = 90;    // degrees
//    public static double MAX_ANG_VEL = Math.toRadians(180); // rad/s
//    public static double MAX_ANG_ACCEL = Math.toRadians(180); // rad/s^2
//
//    private enum TestMode {
//        NONE,
//        STRAIGHT,
//        STRAFE,
//        TURN,
//        SPLINE
//    }
//
//    private TestMode currentMode = TestMode.NONE;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        DriveTrainRR drive = new DriveTrainRR(hardwareMap);
//
//        telemetry.addLine("Press A for Straight Test");
//        telemetry.addLine("Press B for Strafe Test");
//        telemetry.addLine("Press X for Turn Test");
//        telemetry.addLine("Press Y for Spline Test");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            checkGamepadInput(gamepad1);
//
//            switch (currentMode) {
//                case STRAIGHT:
//                    runStraightTest(drive, telemetry);
//                    currentMode = TestMode.NONE;
//                    break;
//                case STRAFE:
//                    runStrafeTest(drive, telemetry);
//                    currentMode = TestMode.NONE;
//                    break;
//                case TURN:
//                    runTurnTest(drive, telemetry);
//                    currentMode = TestMode.NONE;
//                    break;
//                case SPLINE:
//                    runSplineTest(drive, telemetry);
//                    currentMode = TestMode.NONE;
//                    break;
//                default:
//                    // Do nothing
//                    break;
//            }
//
//            telemetry.update();
//            idle();
//        }
//    }
//
//    private void checkGamepadInput(Gamepad gamepad) {
//        if (gamepad.a) {
//            currentMode = TestMode.STRAIGHT;
//        } else if (gamepad.b) {
//            currentMode = TestMode.STRAFE;
//        } else if (gamepad.x) {
//            currentMode = TestMode.TURN;
//        } else if (gamepad.y) {
//            currentMode = TestMode.SPLINE;
//        }
//    }
//
//    private void runStraightTest(DriveTrainRR drive, Telemetry telemetry) {
//        telemetry.addLine("Running Straight Test");
//
//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .forward(DISTANCE)
//                .build();
//
//        drive.followTrajectorySequence(trajectory);
//
//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", Math.toDegrees(poseEstimate.getHeading()));
//    }
//
//    private void runStrafeTest(DriveTrainRR drive, Telemetry telemetry) {
//        telemetry.addLine("Running Strafe Test");
//
//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .strafeRight(DISTANCE)
//                .build();
//
//        drive.followTrajectorySequence(trajectory);
//
//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", Math.toDegrees(poseEstimate.getHeading()));
//    }
//
//    private void runTurnTest(DriveTrainRR drive, Telemetry telemetry) {
//        telemetry.addLine("Running Turn Test");
//
//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .turn(Math.toRadians(ANGLE))
//                .build();
//
//        drive.followTrajectorySequence(trajectory);
//
//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("finalHeading", Math.toDegrees(poseEstimate.getHeading()));
//    }
//
//    private void runSplineTest(DriveTrainRR drive, Telemetry telemetry) {
//        telemetry.addLine("Running Spline Test");
//
//        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .splineTo(new Vector2d(30, 30), 0)
//                .waitSeconds(2)
//                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                .build();
//
//        drive.followTrajectorySequence(trajectorySequence);
//
//        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", Math.toDegrees(poseEstimate.getHeading()));
//    }
//}