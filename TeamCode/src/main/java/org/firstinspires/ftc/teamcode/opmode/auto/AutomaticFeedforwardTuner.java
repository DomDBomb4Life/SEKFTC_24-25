//// File: AutomaticFeedforwardTuner.java
//package org.firstinspires.ftc.teamcode.opmode.auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.control.Kinematics;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.robotcore.internal.system.Misc;
//import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
//import org.firstinspires.ftc.teamcode.util.RegressionUtil;
//
//import java.util.ArrayList;
//import java.util.List;
//
///**
// * Op mode for computing kV, kStatic, and kA from various drive routines.
// */
//@Config
//@Autonomous(group = "drive")
//public class AutomaticFeedforwardTuner extends LinearOpMode {
//    public static double MAX_POWER = 0.7;
//    public static double DISTANCE = 100; // in
//    public static double MAX_VEL = 50.0; // inches per second (adjust based on your robot)
//    public static double MAX_ACCEL = 30.0; // inches per second squared (adjust as needed)
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        DriveTrainRR drive = new DriveTrainRR(hardwareMap);
//        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//        NanoClock clock = NanoClock.system();
//
//        telemetry.addLine("Press play to begin the feedforward tuning routine");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        telemetry.clearAll();
//        telemetry.addLine("Would you like to fit kStatic?");
//        telemetry.addLine("Press (A) for yes, (B) for no");
//        telemetry.update();
//
//        boolean fitIntercept = false;
//        while (!isStopRequested()) {
//            if (gamepad1.a) {
//                fitIntercept = true;
//                while (!isStopRequested() && gamepad1.a) {
//                    idle();
//                }
//                break;
//            } else if (gamepad1.b) {
//                while (!isStopRequested() && gamepad1.b) {
//                    idle();
//                }
//                break;
//            }
//            idle();
//        }
//
//        telemetry.clearAll();
//        telemetry.addLine(Misc.formatInvariant(
//                "Place your robot on the field with at least %.2f in of room in front", DISTANCE));
//        telemetry.addLine("Press (A) to begin");
//        telemetry.update();
//
//        while (!isStopRequested() && !gamepad1.a) {
//            idle();
//        }
//        while (!isStopRequested() && gamepad1.a) {
//            idle();
//        }
//
//        telemetry.clearAll();
//        telemetry.addLine("Running...");
//        telemetry.update();
//
//        double maxVel = MAX_VEL;
//        double finalVel = MAX_POWER * maxVel;
//        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
//        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);
//
//        List<Double> timeSamples = new ArrayList<>();
//        List<Double> positionSamples = new ArrayList<>();
//        List<Double> velocitySamples = new ArrayList<>();
//        List<Double> powerSamples = new ArrayList<>();
//
//        drive.setPoseEstimate(new Pose2d());
//
//        double startTime = clock.seconds();
//        while (!isStopRequested()) {
//            double elapsedTime = clock.seconds() - startTime;
//            if (elapsedTime > rampTime) {
//                break;
//            }
//            double vel = accel * elapsedTime;
//            double power = vel / maxVel;
//
//            double voltage = batteryVoltageSensor.getVoltage();
//            double compensatedPower = power * (12.0 / voltage);
//
//            timeSamples.add(elapsedTime);
//            positionSamples.add(drive.getPoseEstimate().getX());
//            velocitySamples.add(vel);
//            powerSamples.add(compensatedPower);
//
//            drive.setDrivePower(new Pose2d(compensatedPower, 0.0, 0.0));
//            drive.update();
//        }
//        drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));
//
//        // Fit the kV and kStatic using the collected data
//        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
//                timeSamples, positionSamples, powerSamples, fitIntercept);
//
//        telemetry.clearAll();
//        telemetry.addLine("Quasi-static ramp up test complete");
//        if (fitIntercept) {
//            telemetry.addLine(Misc.formatInvariant("kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
//                    rampResult.kV, rampResult.kStatic, rampResult.rSquare));
//        } else {
//            telemetry.addLine(Misc.formatInvariant("kV = %.5f (R^2 = %.2f)",
//                    rampResult.kV, rampResult.rSquare));
//        }
//        telemetry.addLine("Would you like to fit kA?");
//        telemetry.addLine("Press (A) for yes, (B) for no");
//        telemetry.update();
//
//        boolean fitAccelFF = false;
//        while (!isStopRequested()) {
//            if (gamepad1.a) {
//                fitAccelFF = true;
//                while (!isStopRequested() && gamepad1.a) {
//                    idle();
//                }
//                break;
//            } else if (gamepad1.b) {
//                while (!isStopRequested() && gamepad1.b) {
//                    idle();
//                }
//                break;
//            }
//            idle();
//        }
//
//        if (fitAccelFF) {
//            telemetry.clearAll();
//            telemetry.addLine("Place the robot back in its starting position");
//            telemetry.addLine("Press (A) to continue");
//            telemetry.update();
//
//            while (!isStopRequested() && !gamepad1.a) {
//                idle();
//            }
//            while (!isStopRequested() && gamepad1.a) {
//                idle();
//            }
//
//            telemetry.clearAll();
//            telemetry.addLine("Running...");
//            telemetry.update();
//
//            double maxPowerTime = DISTANCE / maxVel;
//
//            timeSamples.clear();
//            positionSamples.clear();
//            velocitySamples.clear();
//            powerSamples.clear();
//
//            drive.setPoseEstimate(new Pose2d());
//            drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));
//
//            startTime = clock.seconds();
//            while (!isStopRequested()) {
//                double elapsedTime = clock.seconds() - startTime;
//                if (elapsedTime > maxPowerTime) {
//                    break;
//                }
//
//                double voltage = batteryVoltageSensor.getVoltage();
//                double compensatedPower = MAX_POWER * (12.0 / voltage);
//
//                timeSamples.add(elapsedTime);
//                positionSamples.add(drive.getPoseEstimate().getX());
//                powerSamples.add(compensatedPower);
//
//                drive.update();
//            }
//            drive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));
//
//            // Fit the kA using the collected data
//            RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(
//                    timeSamples, positionSamples, powerSamples, rampResult);
//
//            telemetry.clearAll();
//            telemetry.addLine("Constant power test complete");
//            telemetry.addLine(Misc.formatInvariant("kA = %.5f (R^2 = %.2f)",
//                    accelResult.kA, accelResult.rSquare));
//            telemetry.update();
//        }
//
//        while (!isStopRequested()) {
//            idle();
//        }
//    }
//}