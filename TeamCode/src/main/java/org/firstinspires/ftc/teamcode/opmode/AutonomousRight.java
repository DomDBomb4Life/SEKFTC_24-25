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
 * Autonomous for the RIGHT starting position.
 * Removed team color logic and menu selection.
 */
@Autonomous(name = "Autonomous Right Start")
public class AutonomousRight extends OpMode {
    private Robot robot;
    private DriveTrainRR driveTrain;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true);
        driveTrain = new DriveTrainRR(hardwareMap);

        // Set initial pose for RIGHT start
        driveTrain.setPoseEstimate(FieldConstants.RIGHT_START);

        telemetry.addLine("Initialized Right Auto");
    }

    @Override
    public void start() {
        Pose2d pushPos1 = new Pose2d(-36.0, 36.0, Math.toRadians(180));
        Pose2d pushPos2 = new Pose2d(-47.0, 12.0, Math.toRadians(180));
        Pose2d pushPos3 = new Pose2d(-55.0, 12.0, Math.toRadians(180));
        Pose2d pushPos4 = new Pose2d(-63.0, 12.0, Math.toRadians(180));
        // Example logic for right start:
        // Just an example. Adjust as needed. 
        TrajectorySequenceBuilder seqBuilder = driveTrain.trajectorySequenceBuilder(FieldConstants.RIGHT_START);

        // Score specimen at SPECIMEN_SCORING_POSITION
        seqBuilder.lineToLinearHeading(FieldConstants.SPECIMEN_SCORING_POSITION);
        seqBuilder.addTemporalMarkerOffset(-1.0,() -> {
            robot.setState(robot.scoringSpecimenState);
        });
        seqBuilder.waitSeconds(3);
        seqBuilder.addTemporalMarkerOffset(-2.0,() -> {
            robot.onRightTriggerPressed();
        });
        seqBuilder.addTemporalMarkerOffset(0.3,() -> {
            robot.onPrimaryButtonPressed();
        });
        seqBuilder.back(2);
        seqBuilder.waitSeconds(1);
        seqBuilder.addDisplacementMarker(() -> {
            robot.setState(robot.idleState);
        });

        // Push the alliance samples into observation zone
        seqBuilder.back(12);
        seqBuilder.lineToLinearHeading(pushPos1);
        seqBuilder.strafeLeft(24);
        seqBuilder.lineToLinearHeading(pushPos2);

        //Push first sample
        seqBuilder.strafeRight(48);
        seqBuilder.strafeLeft(48);
        seqBuilder.lineToLinearHeading(pushPos3);

        //Push second sample
        seqBuilder.strafeRight(48);
        seqBuilder.strafeLeft(48);
        seqBuilder.lineToLinearHeading(pushPos4);

        //Push second sample
        seqBuilder.strafeRight(48);
        seqBuilder.strafeLeft(24);

        //Park w/ 3 sec on timer
        seqBuilder.addTemporalMarker(27, () -> {});
        seqBuilder.strafeRight(24);

        TrajectorySequence sequence = seqBuilder.build();
        driveTrain.followTrajectorySequenceAsync(sequence);
    }

    @Override
    public void loop() {
        driveTrain.update();
        robot.update();

        telemetry.addData("Current Pose", driveTrain.getPoseEstimate());
        telemetry.addData("Current Robot State", robot.getCurrentStateName());
        telemetry.addData("Current Robot Substate", robot.getCurrentSubstate());
        telemetry.update();
    }
}