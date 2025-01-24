package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.FieldConstants;

/**
 * Autonomous for the RIGHT starting position (expanded, no loops).
 * We remove the for-loop and explicitly define each push position or step.
 */
@Autonomous(name = "Autonomous Right Start (Expanded)")
public class AutonomousRight extends OpMode {
    private Robot robot;
    private DriveTrainRR driveTrain;

    // Example final positions for pushing or location references
    private static final Pose2d PUSH_POS_1 = new Pose2d(-36.0, 36.0, Math.toRadians(180));
    private static final Pose2d PUSH_POS_2 = new Pose2d(-47.0, 12.0, Math.toRadians(180));
    private static final Pose2d PUSH_POS_3 = new Pose2d(-55.0, 12.0, Math.toRadians(180));
    private static final Pose2d PUSH_POS_4 = new Pose2d(-63.0, 12.0, Math.toRadians(180));

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true);
        driveTrain = new DriveTrainRR(hardwareMap);

        // Set initial pose for RIGHT start
        driveTrain.setPoseEstimate(FieldConstants.RIGHT_START);

        telemetry.addLine("Initialized Right Auto (Expanded Version)");
    }

    @Override
    public void start() {
        TrajectorySequenceBuilder seqBuilder = driveTrain.trajectorySequenceBuilder(FieldConstants.RIGHT_START);

        // 1) Score specimen at SPECIMEN_SCORING_POSITION
        seqBuilder.lineToLinearHeading(FieldConstants.SPECIMEN_SCORING_POSITION);
        seqBuilder.addTemporalMarkerOffset(-2.0, () -> {
            robot.setState(robot.scoringSpecimenState);
        });
        // Let the sub-state run
        seqBuilder.waitSeconds(3);
        // Trigger claw open or a final action
        seqBuilder.addTemporalMarkerOffset(-1.5, () -> {
            robot.onRightTriggerPressed();
        });
        // Move back a little
        seqBuilder.back(2);
        seqBuilder.waitSeconds(0.7);

        // Return to idle
        seqBuilder.addDisplacementMarker(() -> {
            robot.setState(robot.idleState);
        });

        // 2) Move back further & line up
        seqBuilder.back(10);

        // 3) Go to push pos #1
        seqBuilder.lineToLinearHeading(PUSH_POS_1);
        // Possibly strafe or wait
        seqBuilder.strafeLeft(24);

        // 4) push pos #2
        seqBuilder.lineToLinearHeading(PUSH_POS_2);
        // push first sample
        seqBuilder.strafeRight(42);
        seqBuilder.strafeLeft(42);

        // 5) push pos #3
        seqBuilder.lineToLinearHeading(PUSH_POS_3);
        // push second sample
        seqBuilder.strafeRight(42);
        seqBuilder.strafeLeft(42);

        // 6) push pos #4
        seqBuilder.lineToLinearHeading(PUSH_POS_4);
        // final push
        seqBuilder.strafeRight(42);

        // Possibly park or do last second manipulations
        seqBuilder.addTemporalMarker(27, () -> {
            robot.arm.moveToAngle(137);
        });

        // Build & run
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