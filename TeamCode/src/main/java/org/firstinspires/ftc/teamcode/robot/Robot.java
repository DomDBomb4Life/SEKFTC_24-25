// File: Robot.java
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.robot.states.HomeState;
import org.firstinspires.ftc.teamcode.robot.states.ScoringBasketState;
import org.firstinspires.ftc.teamcode.robot.states.ScoringSpecimenState;
import org.firstinspires.ftc.teamcode.robot.states.ObservationState;
import org.firstinspires.ftc.teamcode.robot.states.IdleState;
import org.firstinspires.ftc.teamcode.robot.states.LevelOneAscentState;
import org.firstinspires.ftc.teamcode.robot.states.PickupState; // Import the new state

/**
 * The Robot class encapsulates all the subsystems and manages state transitions.
 */
public class Robot {

    // Subsystems
    public final ViperLift viperLift;
    public final Arm arm;
    public final Claw claw;
    public final Wrist wrist;

    // State instances
    public HomeState homeState;
    public ScoringBasketState scoringBasketState;
    public ScoringSpecimenState scoringSpecimenState;
    public ObservationState observationState;
    public IdleState idleState;
    public LevelOneAscentState levelOneAscentState;
    public PickupState pickupState; // New PickupState instance

    // Current state
    public enum State {
        IDLE,
        HOME,
        SCORING,
        SCORING_SPECIMEN,
        OBSERVATION,
        LEVEL_ONE_ASCENT,
        PICKUP // New state added
    }

    public State currentState = State.IDLE;

    // Constructor
    public Robot(HardwareMap hardwareMap) {
        // Initialize subsystems
        claw = new Claw(hardwareMap); // Initialize Claw first
        wrist = new Wrist(hardwareMap, claw); // Pass Claw to Wrist
        arm = new Arm(hardwareMap);
        viperLift = new ViperLift(hardwareMap);

        // Initialize states
        homeState = new HomeState(viperLift, arm, wrist, claw);
        scoringBasketState = new ScoringBasketState(viperLift, arm, wrist, claw);
        scoringSpecimenState = new ScoringSpecimenState(viperLift, arm, wrist, claw);
        observationState = new ObservationState(viperLift, arm, wrist, claw);
        idleState = new IdleState(viperLift, arm, wrist, claw);
        levelOneAscentState = new LevelOneAscentState(viperLift, arm);
        pickupState = new PickupState(viperLift, arm, wrist, claw); // Initialize new state

        // Activate initial state
        idleState.activate();
    }

    // Update method called periodically
    public void update() {
        // Update subsystems and states
        switch (currentState) {
            case HOME:
                homeState.update();
                break;

            case SCORING:
                scoringBasketState.update();
                if (scoringBasketState.isCompleted()) {
                    setState(State.IDLE);
                }
                break;

            case SCORING_SPECIMEN:
                scoringSpecimenState.update();
                if (scoringSpecimenState.isCompleted()) {
                    setState(State.IDLE);
                }
                break;

            case OBSERVATION:
                observationState.update();
                break;

            case LEVEL_ONE_ASCENT:
                levelOneAscentState.update();
                // Do not set to IDLE when completed, as per your request
                break;

            case PICKUP:
                pickupState.update();
                // Decide when to transition out of PICKUP state if needed
                break;

            case IDLE:
                idleState.update();
                break;

            default:
                break;
        }
    }

    // Method to set the current state
    public void setState(State newState) {
        if (currentState != newState) {
            // Deactivate current state if necessary
            switch (currentState) {
                case HOME:
                    homeState.deactivate();
                    break;
                case PICKUP:
                    pickupState.deactivate();
                    break;
                // Add deactivation for other states if needed
                default:
                    break;
            }

            currentState = newState;
            switch (newState) {
                case HOME:
                    homeState.start();
                    break;

                case SCORING:
                    scoringBasketState.start();
                    break;

                case SCORING_SPECIMEN:
                    scoringSpecimenState.start();
                    break;

                case OBSERVATION:
                    observationState.activate();
                    break;

                case LEVEL_ONE_ASCENT:
                    levelOneAscentState.start();
                    break;

                case PICKUP:
                    pickupState.activate();
                    break;

                case IDLE:
                    idleState.activate();
                    break;

                default:
                    break;
            }
        } else {
            // If the same state button is pressed again, toggle back to IDLE
            setState(State.IDLE);
        }
    }

    // Method to get current substate
    public String getCurrentSubstate() {
        switch (currentState) {
            case HOME:
                return homeState.getCurrentStep().toString();
            case SCORING:
                return scoringBasketState.getCurrentStep().toString();
            case SCORING_SPECIMEN:
                return scoringSpecimenState.getCurrentStep().toString();
            case OBSERVATION:
                return observationState.isActive() ? "ACTIVE" : "INACTIVE";
            case LEVEL_ONE_ASCENT:
                return levelOneAscentState.getCurrentStep().toString();
            case PICKUP:
                return pickupState.isReadyToPickUp() ? "READY" : "MOVING";
            case IDLE:
            default:
                return "NONE";
        }
    }

    // Methods to handle button presses
    public void onHomeButtonPressed() {
        if (currentState != State.HOME) {
            setState(State.HOME);
        } else {
            setState(State.IDLE);
        }
    }

    public void onScoringButtonPressed() {
        setState(State.SCORING);
    }

    public void onScoringSpecimenButtonPressed() {
        if (currentState == State.SCORING_SPECIMEN) {
            scoringSpecimenState.onPrimaryButtonPressed();
        } else {
            setState(State.SCORING_SPECIMEN);
        }
    }

    public void onObservationButtonPressed() {
        setState(State.OBSERVATION);
    }

    public void onClawToggleButtonPressed() {
        if (claw.isClosed()) {
            claw.open();
        } else {
            claw.close();
        }
    }

    // Method to handle right trigger press
    public void onRightTriggerPressed() {
        if (currentState == State.HOME) {
            homeState.onRightTriggerPressed();
        } else if (currentState == State.LEVEL_ONE_ASCENT) {
            levelOneAscentState.onRightTriggerPressed();
        } else if (currentState == State.SCORING_SPECIMEN) {
            scoringSpecimenState.onRightTriggerPressed();
        }
    }

    // Method to handle Level One Ascent button press
    public void onLevelOneAscentButtonPressed() {
        setState(State.LEVEL_ONE_ASCENT);
    }

    // Method to handle Pickup button press
    public void onPickupButtonPressed() {
        if (currentState != State.PICKUP) {
            setState(State.PICKUP);
        } else {
            setState(State.IDLE);
        }
    }
}