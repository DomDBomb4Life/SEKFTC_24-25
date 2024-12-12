// // File: AutonomousOpModeV2.java
// package org.firstinspires.ftc.teamcode.opmode;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;
// import org.firstinspires.ftc.teamcode.robot.Robot;

// import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// import com.acmerobotics.roadrunner.geometry.Pose2d;

// import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
// import org.firstinspires.ftc.teamcode.util.FieldConstants;
// import org.firstinspires.ftc.teamcode.util.FieldConstants.TeamColor;
// import org.firstinspires.ftc.teamcode.util.FieldConstants.StartingPosition;

// /**
//  * Autonomous Op Mode that allows pre-match configuration via Gamepad 1.
//  */
// @Autonomous(name = "Autonomous Op Mode V3")
// public class AutonomousOpModeV3 extends OpMode {

//     private TeamColor teamColor = TeamColor.BLUE;
//     private StartingPosition startingPosition = StartingPosition.LEFT;

//     private Robot robot;
//     private DriveTrainRR driveTrain;

//     private boolean isInitialized = false;

//     @Override
//     public void init() {
//         robot = new Robot(hardwareMap, true); // Pass true for isAutonomous
//         driveTrain = new DriveTrainRR(hardwareMap);

//         // Initialize pose estimate
//         Pose2d startPose = FieldConstants.getStartingPose(teamColor, startingPosition);
//         driveTrain.setPoseEstimate(startPose);

//         telemetry.addLine("Initialized");
//     }

//     @Override
//     public void init_loop() {
//         handleMenuSelection();
//         telemetry.addData("Team Color", teamColor);
//         telemetry.addData("Starting Position", startingPosition);
//         telemetry.update();
//     }

//     @Override
//     public void start() {
//         if (startingPosition == StartingPosition.LEFT) {
//             executeLeftStartingPosition();
//         } else {
//             executeRightStartingPosition();
//         }
//     }

//     @Override
//     public void loop() {
//         driveTrain.update();
//         robot.update();

//         telemetry.addData("Current Pose", driveTrain.getPoseEstimate());
//         telemetry.addData("Current Robot State", robot.getCurrentStateName());
//         telemetry.addData("Current Robot Substate", robot.getCurrentSubstate());
//         telemetry.update();
//     }

//     private void handleMenuSelection() {
//         if (gamepad1.dpad_up) {
//             teamColor = (teamColor == TeamColor.RED) ? TeamColor.BLUE : TeamColor.RED;
//         }

//         if (gamepad1.dpad_left) {
//             if (startingPosition == StartingPosition.LEFT){
//                 startingPosition = StartingPosition.RIGHT;
//             }else{
//                 startingPosition = StartingPosition.LEFT;
//             }
//         }
//     }

//     private void executeLeftStartingPosition() {
//         Pose2d startPose = FieldConstants.getStartingPose(teamColor, StartingPosition.LEFT);
//         Pose2d netPosition = FieldConstants.getNetPosition(teamColor);
//         Pose2d ascentZonePosition = FieldConstants.getAscentZonePosition(teamColor);
//         Pose2d[] samplePositions = FieldConstants.getSamplePositions(teamColor);

//         // Build the trajectory sequence
//         TrajectorySequenceBuilder seqBuilder = driveTrain.trajectorySequenceBuilder(FieldConstants.LEFT_START);

//         // Drive to Net Position
//         seqBuilder.lineToLinearHeading(netPosition);

//         // Add marker to set robot state to scoringBasketState
//         seqBuilder.addTemporalMarkerOffset(-1, () -> {
//             robot.setState(robot.scoringBasketState);
//         });
//         seqBuilder.waitSeconds(5);

//         // For each sample position
//         for (Pose2d samplePosition : samplePositions) {
//             // Drive to Sample Position
//             seqBuilder.lineToLinearHeading(samplePosition);

//             // Add marker to set robot state to pickupState
//             seqBuilder.addTemporalMarkerOffset(-0.5, () -> {
//                 robot.setState(robot.homeState);
//             });

//             // Wait for robot to reach "WAIT_FOR_DRIVE_FORWARD"
//             // Adjust timing as necessary
//             seqBuilder.waitSeconds(1.0);

//             // Drive forward 4 inches

//             // Add marker to call robot.pickupState.onDriveForwardComplete()
//             seqBuilder.addTemporalMarkerOffset(0.0,() -> {
//                 robot.onRightTriggerPressed();
//             });

//             // Wait for robot state completion
//             seqBuilder.waitSeconds(1.0);

//             // Drive back to Net Position
//             seqBuilder.lineToLinearHeading(netPosition);

//             // Add marker to set robot state to scoringBasketState
//             seqBuilder.addTemporalMarkerOffset(-1, () -> {
//                 robot.setState(robot.scoringBasketState);
//             });
//             seqBuilder.waitSeconds(5);
//         }


//         // Add marker to set robot state to levelOneAscentState
//         seqBuilder.addTemporalMarkerOffset(0, () -> {
//             robot.setState(robot.levelOneAscentState);
//         });
        
//         // Drive to Ascent Zone Position
//         seqBuilder.lineToLinearHeading(ascentZonePosition);
//         seqBuilder.waitSeconds(1);
//         seqBuilder.forward(4);

//         seqBuilder.addTemporalMarkerOffset(0.0,() -> {
//             robot.onRightTriggerPressed();
//         });


//         // Build the trajectory sequence
//         TrajectorySequence sequence = seqBuilder.build();

//         // Start following the trajectory sequence asynchronously
//         driveTrain.followTrajectorySequenceAsync(sequence);
//     }

//     private void executeRightStartingPosition() {
//         Pose2d startPose = FieldConstants.getStartingPose(teamColor, StartingPosition.RIGHT);
//         Pose2d[] samplePositions = FieldConstants.getAllianceSamplePositions(teamColor);
//         Pose2d specimenScoringPosition = FieldConstants.getSpecimenScoringPosition(teamColor);
//         Pose2d pushPos1 = new Pose2d(-36.0, 36.0, Math.toRadians(180));
//         Pose2d pushPos2 = new Pose2d(-47.0, 12.0, Math.toRadians(180));
//         Pose2d pushPos3 = new Pose2d(-55.0, 12.0, Math.toRadians(180));
//         Pose2d pushPos4 = new Pose2d(-63.0, 12.0, Math.toRadians(180));




//         TrajectorySequenceBuilder seqBuilder = driveTrain.trajectorySequenceBuilder(FieldConstants.RIGHT_START);

//         //Score the specimen
//         seqBuilder.lineToLinearHeading(specimenScoringPosition);
//         seqBuilder.addTemporalMarkerOffset(-1.0,() -> {
//             robot.setState(robot.scoringSpecimenState);
//         });
//         seqBuilder.waitSeconds(3);
//         seqBuilder.addTemporalMarkerOffset(-2.0,() -> {
//             robot.onRightTriggerPressed();
//         });
//         seqBuilder.addTemporalMarkerOffset(0.3,() -> {
//             robot.onPrimaryButtonPressed();
//         });
//         seqBuilder.back(2);
//         seqBuilder.waitSeconds(1);
//         seqBuilder.addDisplacementMarker(() ->{
//             robot.setState(robot.idleState);
//         });

//         // Push the alliance samples into observation zone
//         seqBuilder.back(12);
//         seqBuilder.lineToLinearHeading(pushPos1);
//         seqBuilder.strafeLeft(24);
//         seqBuilder.lineToLinearHeading(pushPos2);

//         //Push first sample
//         seqBuilder.strafeRight(48);
//         seqBuilder.strafeLeft(48);
//         seqBuilder.lineToLinearHeading(pushPos3);

//         //Push second sample
//         seqBuilder.strafeRight(48);
//         seqBuilder.strafeLeft(48);
//         seqBuilder.lineToLinearHeading(pushPos4);

//         //Push second sample
//         seqBuilder.strafeRight(48);
//         seqBuilder.strafeLeft(24);

//         //Park w/ 3 sec on timer
//         seqBuilder.addTemporalMarker(27, () -> {});
//         seqBuilder.strafeRight(24);

//         TrajectorySequence sequence = seqBuilder.build();
//         driveTrain.followTrajectorySequenceAsync(sequence);


//         telemetry.update();
//     }

//     // Helper method to simulate sleep in OpMode

// }