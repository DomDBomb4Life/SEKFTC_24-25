//// File: ManualFeedforwardTuner.java
//package org.firstinspires.ftc.teamcode.opmode.auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.kinematics.Kinematics;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.profile.*;
//import com.acmerobotics.roadrunner.util.NanoClock;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
//
//import java.util.Objects;
//
///**
// * Op mode for manually tuning feedforward coefficients.
// */
//@Config
//@TeleOp(group = "drive")
//public class ManualFeedforwardTuner extends LinearOpMode {
//    public static double DISTANCE = 60; // in
//    public static double MAX_VEL = 50.0; // in/s
//    public static double MAX_ACCEL = 30.0; // in/s^2
//    public static double kV = 0.0; // Set initial value
//    public static double kA = 0.0; // Set initial value
//    public static double kStatic = 0.0; // Set initial value
//
//    enum Mode {
//        DRIVER_MODE,
//        TUNING_MODE
//    }
//
//    private Mode mode;
//
//    @Override
//    public void runOpMode() {
//        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        DriveTrainRR drive = new DriveTrainRR(hardwareMap);
//
//        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
//
//        mode = Mode.TUNING_MODE;
//
//        NanoClock clock = NanoClock.system();
//
//        telemetry.addLine("Ready!");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        boolean movingForwards = true;
//        MotionProfile activeProfile = generateProfile(true);
//        double profileStart = clock.seconds();
//
//        while (!isStopRequested()) {
//            telemetry.addData("mode", mode);
//
//            switch (mode) {
//                case TUNING_MODE:
//                    if (gamepad1.a) {
//                        mode = Mode.DRIVER_MODE;
//                    }
//
//                    // Calculate and set the motor power
//                    double profileTime = clock.seconds() - profileStart;
//
//                    if (profileTime > activeProfile.duration()) {
//                        // Generate a new profile
//                        movingForwards = !movingForwards;
//                        activeProfile = generateProfile(movingForwards);
//                        profileStart = clock.seconds();
//                    }
//
//                    MotionState motionState = activeProfile.get(profileTime);
//                    double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);
//
//                    double voltage = batteryVoltageSensor.getVoltage();
//                    double compensatedPower = targetPower * (12.0 / voltage);
//
//                    drive.setDrivePower(new Pose2d(compensatedPower, 0, 0));
//                    drive.update();
//
//                    Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null.");
//                    double currentVelo = poseVelo.getX();
//
//                    // Update telemetry
//                    telemetry.addData("targetVelocity", motionState.getV());
//                    telemetry.addData("measuredVelocity", currentVelo);
//                    telemetry.addData("error", motionState.getV() - currentVelo);
//                    break;
//                case DRIVER_MODE:
//                    if (gamepad1.b) {
//                        mode = Mode.TUNING_MODE;
//                        movingForwards = true;
//                        activeProfile = generateProfile(movingForwards);
//                        profileStart = clock.seconds();
//                    }
//
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y,
//                                    -gamepad1.left_stick_x,
//                                    -gamepad1.right_stick_x
//                            )
//                    );
//                    break;
//            }
//
//            telemetry.update();
//        }
//    }
//
//    private MotionProfile generateProfile(boolean movingForward) {
//        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
//        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
//        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
//    }
//}