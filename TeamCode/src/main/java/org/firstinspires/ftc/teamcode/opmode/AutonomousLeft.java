package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveTrainRR.getAccelerationConstraint;

import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;


import org.firstinspires.ftc.teamcode.util.FieldConstants;

/**
 * Autonomous for the LEFT starting position, fully expanded with no for-loops
 * so we can precisely tune each step and minimize downtime.
 */
@Autonomous(name = "Autonomous Left Start")
public class AutonomousLeft extends OpMode {
    private Robot robot;
    private DriveTrainRR driveTrain;
    // Example explicit sample positions (replace with actual from FieldConstants or your config)
    private static final Pose2d SAMPLE_1 = FieldConstants.SAMPLE_POSITIONS[0];
    private static final Pose2d SAMPLE_2 = FieldConstants.SAMPLE_POSITIONS[1];
    private static final Pose2d SAMPLE_3 = FieldConstants.SAMPLE_POSITIONS[2];

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true);
        driveTrain = new DriveTrainRR(hardwareMap);

        // Example of pre-positioning the arm/wrist
        robot.arm.moveToAngle(139);
        robot.wrist.setAngle(18);
        robot.claw.close();

        // Set initial pose for LEFT start
        driveTrain.setPoseEstimate(FieldConstants.LEFT_START);

        telemetry.addLine("Initialized Left Auto (Expanded Version, No For-Loop)");
        telemetry.addData("Claw Target", robot.claw.getTargetAngle());
        telemetry.addData("Claw Servo Position", robot.claw.getPosition());    }

    @Override
    public void start() {
        // Build trajectory for LEFT start
        TrajectorySequenceBuilder seqBuilder = driveTrain.trajectorySequenceBuilder(FieldConstants.LEFT_START);

        // 1) Drive to specimen scoring position
        seqBuilder.lineToLinearHeading(FieldConstants.SPECIMEN_SCORING_POSITION_LEFT);

        // Start scoring specimen slightly early
        seqBuilder.addTemporalMarkerOffset(-2.0, () -> {
            robot.setState(robot.scoringSpecimenState);
        });
        // Wait 3s for the sub-state to do partial actions
        seqBuilder.waitSeconds(3);

        // Optionally close claw or trigger further action
        seqBuilder.addTemporalMarkerOffset(-2.2, () -> {
            robot.onRightTriggerPressed();
        });
        seqBuilder.addTemporalMarkerOffset(-0.5,() -> {
            robot.viperLift.moveToPosition(0);
        });
        seqBuilder.addTemporalMarkerOffset(0, () -> {
            seqBuilder.setAccelConstraint(DriveTrainRR.getAccelerationConstraint(10.0));
        });

        // 2) SAMPLE #1
        seqBuilder.lineToLinearHeading(SAMPLE_1);
        // Simulate pickup process
        seqBuilder.addTemporalMarkerOffset(-1.2, () -> {
            robot.setState(robot.homeState);
        });
        seqBuilder.addTemporalMarkerOffset(-0.25, () -> {
            // E.g., press right trigger to close claw
            robot.onRightTriggerPressed();
        });
        seqBuilder.waitSeconds(1.2);

        // Return to net
        seqBuilder.lineToLinearHeading(FieldConstants.NET_POSITION);
        seqBuilder.addTemporalMarkerOffset(-1.2, () -> {
            robot.setState(robot.scoringBasketState);
        });
        seqBuilder.waitSeconds(2.5);


        // 3) SAMPLE #2
        seqBuilder.addTemporalMarkerOffset(0.0, () -> {
            robot.viperLift.moveToPosition(442);
        });
        seqBuilder.lineToLinearHeading(SAMPLE_2);
        seqBuilder.addTemporalMarkerOffset(0.0, () -> {
            robot.setState(robot.homeState);
            robot.viperLift.moveToPosition(0);
        });
        seqBuilder.waitSeconds(1.0);
        seqBuilder.addTemporalMarkerOffset(0.0, () -> {
            robot.onRightTriggerPressed();
        });
        seqBuilder.waitSeconds(1.0);

        // Return to net
        seqBuilder.lineToLinearHeading(FieldConstants.NET_POSITION);
        seqBuilder.addTemporalMarkerOffset(-1.2, () -> {
            robot.setState(robot.scoringBasketState);
        });
        seqBuilder.waitSeconds(2.5);

        // 4) SAMPLE #3
        seqBuilder.addTemporalMarkerOffset(0.0, () -> {
            robot.viperLift.moveToPosition(442);
        });
//        seqBuilder.lineToLinearHeading(SAMPLE_3);
//        seqBuilder.addTemporalMarkerOffset(0.0, () -> {
//            robot.setState(robot.homeState);
//            robot.viperLift.moveToPosition(0);
//        });
//        seqBuilder.waitSeconds(0.5);
//        seqBuilder.turn(Math.toRadians(16));
//        seqBuilder.waitSeconds(0.5);
//
//
//        seqBuilder.addTemporalMarkerOffset(0.0, () -> {
//            robot.onRightTriggerPressed();
//        });
//        seqBuilder.waitSeconds(1.0);
//
//        // Return to net
//        seqBuilder.lineToLinearHeading(FieldConstants.NET_POSITION);
//        seqBuilder.addTemporalMarkerOffset(-1.2, () -> {
//            robot.setState(robot.scoringBasketState);
//        });
//        seqBuilder.waitSeconds(2.5);



        seqBuilder.addTemporalMarkerOffset(0, () -> {
            seqBuilder.resetAccelConstraint();
        });

        seqBuilder.addTemporalMarkerOffset(0.0, () -> {
            robot.viperLift.moveToPosition(0);
        });
         // 5) Move to ascent zone
         seqBuilder.addTemporalMarkerOffset(-0.8, () -> {
             robot.setState(robot.levelOneAscentState);
         });

         seqBuilder.lineToLinearHeading(FieldConstants.ASCENT_ZONE_POSITION);

         // Possibly adjust arm/wrist
         seqBuilder.addTemporalMarkerOffset(0.2, () -> {
             robot.arm.moveToAngle(75);
             robot.wrist.setAngle(90);
         });

        // Build and start
        TrajectorySequence sequence = seqBuilder.build();
        driveTrain.followTrajectorySequenceAsync(sequence);
    }

    @Override
    public void loop() {
        driveTrain.update();
        robot.update();

        telemetry.addData("Current Pose", driveTrain.getPoseEstimate());
        telemetry.addData("Robot State", robot.getCurrentStateName());
        telemetry.addData("Substate", robot.getCurrentSubstate());
        telemetry.addData("Arm Angle", robot.arm.getCurrentAngle());
        telemetry.addData("Arm Target", robot.arm.getTargetAngle());
        telemetry.update();
    }
}