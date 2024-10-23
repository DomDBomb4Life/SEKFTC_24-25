// File: Robot.java
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.robot.states.HomeState;
import org.firstinspires.ftc.teamcode.robot.states.HangingState;
import org.firstinspires.ftc.teamcode.robot.states.ScoringBasketState;
import org.firstinspires.ftc.teamcode.robot.states.IdleState;
import org.firstinspires.ftc.teamcode.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.localization.Localization;
import org.firstinspires.ftc.teamcode.aimbot.AimbotController;
import org.firstinspires.ftc.teamcode.drive.DriveTrain;

public class Robot {

    // Hardware components
    public ViperLift viperLift;
    public Arm arm;
    public Claw claw;
    public Wrist wrist;

    // State variables
    public RobotState currentState;

    // State instances
    public HomeState homeState;
    public HangingState hangingState;
    public ScoringBasketState scoringBasketState;
    public IdleState idleState;

    // New components
    public VisionSystem visionSystem;
    public Localization localization;
    public AimbotController aimbotController;

    // Constructor
    public Robot(HardwareMap hardwareMap, DriveTrain driveTrain) {
        // Initialize hardware components
        viperLift = new ViperLift(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);

        // Initialize states
        homeState = new HomeState(viperLift, arm, wrist);
        hangingState = new HangingState(viperLift, arm);
        scoringBasketState = new ScoringBasketState(viperLift, arm, wrist, claw);
        idleState = new IdleState(viperLift, arm);

        // Set initial state
        currentState = RobotState.IDLE;
        idleState.activate();

        // Initialize vision system
        visionSystem = new VisionSystem(hardwareMap);

        // Initialize localization
        localization = new Localization(hardwareMap, visionSystem);

        // Initialize aimbot controller
        aimbotController = new AimbotController(localization, driveTrain);
    }

    // Update method called periodically
    public void update() {
        // Update vision system
        visionSystem.update();

        // Update localization
        localization.update();

        // Update aimbot
        aimbotController.update();

        // Update states
        switch (currentState) {
            case HOME:
                homeState.update();
                break;

            case HANGING:
                hangingState.update();
                break;

            case SCORING_BASKET:
                scoringBasketState.update();
                if (scoringBasketState.getCurrentStep() == ScoringBasketState.Step.COMPLETED) {
                    // Transition to idle state after scoring process is completed
                    currentState = RobotState.IDLE;
                    idleState.activate();
                }
                break;

            case IDLE:
                idleState.update();
                break;

            default:
                // Handle other states or idle behavior
                break;
        }
    }

    // Methods to handle aimbot
    public void activateAimbot(Pose2d targetPose) {
        aimbotController.activate(targetPose);
    }

    public void deactivateAimbot() {
        aimbotController.deactivate();
    }

    // Methods to handle button presses

    // Hanging State
    public void onHangingButtonPressed() {
        if (currentState != RobotState.HANGING) {
            currentState = RobotState.HANGING;
            hangingState.start();
        } else {
            // Progress through steps
            hangingState.progress();
        }
    }

    // Claw Toggle
    public void onClawToggleButtonPressed() {
        if (currentState == RobotState.HOME) {
            toggleClaw();
        }
        // Else, do nothing
    }

    // Scoring Basket State
    public void onScoringBasketButtonPressed() {
        if (currentState != RobotState.SCORING_BASKET) {
            currentState = RobotState.SCORING_BASKET;
            scoringBasketState.start();
        } else {
            // Progress to next step
            scoringBasketState.progress();
        }
    }

    // Home State
    public void onHomeButtonPressed() {
        if (currentState != RobotState.HOME) {
            currentState = RobotState.HOME;
            homeState.activate();
        }
    }

    public void onSwitchVariationButtonPressed() {
        if (currentState == RobotState.HOME) {
            homeState.switchVariation();
        }
    }

    // Claw control
    private void toggleClaw() {
        if (claw.isClosed()) {
            claw.open();
        } else {
            claw.close();
        }
    }
}