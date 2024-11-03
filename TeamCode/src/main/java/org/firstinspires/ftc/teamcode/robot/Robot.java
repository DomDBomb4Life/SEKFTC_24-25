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
import org.firstinspires.ftc.teamcode.robot.states.ScoringSpecimenState;
import org.firstinspires.ftc.teamcode.robot.states.ObservationState;
import org.firstinspires.ftc.teamcode.robot.states.IdleState;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;

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
    public HangingState hangingState;
    public ScoringBasketState scoringBasketState;
    public ScoringSpecimenState scoringSpecimenState;
    public ObservationState observationState;
    public IdleState idleState;

    // Drivetrain
    private final DriveTrainRR driveTrain;

    // Current state
    public enum State {
        IDLE,
        HOME,
        HANGING,
        SCORING,
        SCORING_SPECIMEN,
        OBSERVATION
    }

    public State currentState = State.IDLE;

    // Constructor
    public Robot(HardwareMap hardwareMap, DriveTrainRR driveTrain) {
        // Initialize subsystems
        viperLift = new ViperLift(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);

        // Initialize states
        homeState = new HomeState(viperLift, arm, wrist, claw);
        hangingState = new HangingState(viperLift, arm);
        scoringBasketState = new ScoringBasketState(viperLift, arm, wrist, claw);
        scoringSpecimenState = new ScoringSpecimenState(viperLift, arm, wrist, claw);
        observationState = new ObservationState(viperLift, arm, wrist, claw);
        idleState = new IdleState(viperLift, arm, wrist, claw);

        // Activate initial state
        idleState.activate();

        // Initialize drivetrain
        this.driveTrain = driveTrain;
    }

    // Getter for drivetrain
    public DriveTrainRR getDriveTrain() {
        return driveTrain;
    }

    // Update method called periodically
    public void update() {
        // Update subsystems and states
        switch (currentState) {
            case HOME:
                homeState.update();
                break;

            case HANGING:
                hangingState.update();
                break;

            case SCORING:
                scoringBasketState.update();
                if (scoringBasketState.isCompleted()) {
                    setState(State.IDLE);
                }
                break;

            case SCORING_SPECIMEN:
                // Update is handled in TeleOpMode to pass button states
                if (scoringSpecimenState.isCompleted()) {
                    setState(State.IDLE);
                }
                break;

            case OBSERVATION:
                observationState.update();
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
            currentState = newState;
            switch (newState) {
                case HOME:
                    homeState.activate();
                    break;

                case HANGING:
                    hangingState.start();
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
                return homeState.getCurrentVariation().toString();
            case HANGING:
                return hangingState.getCurrentStep().toString();
            case SCORING:
                return scoringBasketState.getCurrentStep().toString();
            case SCORING_SPECIMEN:
                return scoringSpecimenState.getCurrentStep().toString();
            case OBSERVATION:
                return observationState.isActive() ? "ACTIVE" : "INACTIVE";
            case IDLE:
            default:
                return "NONE";
        }
    }

    // Methods to handle button presses
    public void onHomeButtonPressed() {
        setState(State.HOME);
    }

    public void onHangingButtonPressed() {
        setState(State.HANGING);
    }

    public void onScoringButtonPressed() {
        setState(State.SCORING);
    }

    public void onScoringSpecimenButtonPressed() {
        setState(State.SCORING_SPECIMEN);
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

    public void onSwitchVariationButtonPressed() {
        if (currentState == State.HOME) {
            homeState.switchVariation();
        }
    }
}