// File: Robot.java
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.robot.states.HomeState;
import org.firstinspires.ftc.teamcode.robot.states.HangingState;
import org.firstinspires.ftc.teamcode.robot.states.ScoringBasketState;
import org.firstinspires.ftc.teamcode.robot.states.IdleState;

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

    // Constructor
    public Robot(HardwareMap hardwareMap) {
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
    }

    // Update method called periodically
    public void update() {
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