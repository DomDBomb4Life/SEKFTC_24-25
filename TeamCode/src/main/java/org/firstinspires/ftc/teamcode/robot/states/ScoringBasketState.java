// File: ScoringBasketState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

/**
 * Represents the state of the robot when scoring into the basket.
 */
public class ScoringBasketState {

    // Subsystems
    private final ViperLift viperLift;
    private final Arm arm;
    private final Wrist wrist;
    private final Claw claw;

    // Steps in the scoring basket process
    public enum Step {
        LIFT_UP,
        ARM_POSITION,
        OPEN_CLAW,
        ARM_RETURN,
        LIFT_DOWN,
        COMPLETED
    }

    private Step currentStep;

    // Timer for step 3
    private long stepStartTime;
    private static final long STEP_3_DURATION_MS = 1000; // Adjust as needed

    // Constructor
    public ScoringBasketState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.currentStep = Step.LIFT_UP;
    }

    // Start the scoring basket process
    public void start() {
        currentStep = Step.LIFT_UP;
        executeCurrentStep();
    }

    // Progress to the next step if conditions are met
    private void progress() {
        switch (currentStep) {
            case LIFT_UP:
                currentStep = Step.ARM_POSITION;
                break;
            case ARM_POSITION:
                currentStep = Step.OPEN_CLAW;
                break;
            case OPEN_CLAW:
                currentStep = Step.ARM_RETURN;
                break;
            case ARM_RETURN:
                currentStep = Step.LIFT_DOWN;
                break;
            case LIFT_DOWN:
                currentStep = Step.COMPLETED;
                break;
            default:
                break;
        }
        executeCurrentStep();
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case LIFT_UP:
                // Move Viper Lift all the way up
                viperLift.moveToMax();
                break;

            case ARM_POSITION:
                // Move arm and wrist into scoring position
                arm.moveToAngle(180);
                wrist.setAngle(180);
                break;

            case OPEN_CLAW:
                // Open the claw to release the object
                claw.open();
                // Start timer
                stepStartTime = System.currentTimeMillis();
                break;

            case ARM_RETURN:
                // Move arm and wrist back to neutral position
                arm.moveToAngle(90);
                wrist.setAngle(90);
                break;

            case LIFT_DOWN:
                // Lower Viper Lift back to initial position
                viperLift.moveToPosition(0);
                break;

            case COMPLETED:
                // Scoring process completed
                break;

            default:
                break;
        }
    }

    // Update method to be called periodically
    public void update() {
        switch (currentStep) {
            case LIFT_UP:
                if (viperLift.isAtTarget()) {
                    progress();
                }
                break;

            case ARM_POSITION:
                if (arm.isAtTarget() && wrist.isAtTarget()) {
                    progress();
                }
                break;

            case OPEN_CLAW:
                // Wait for timer to complete
                long elapsedTime = System.currentTimeMillis() - stepStartTime;
                if (elapsedTime >= STEP_3_DURATION_MS) {
                    progress();
                }
                break;

            case ARM_RETURN:
                if (arm.isAtTarget() && wrist.isAtTarget()) {
                    progress();
                }
                break;

            case LIFT_DOWN:
                if (viperLift.isAtTarget()) {
                    progress();
                }
                break;

            case COMPLETED:
                // Process completed; optionally reset or stay in this state
                break;

            default:
                break;
        }
    }

    // Get the current step
    public Step getCurrentStep() {
        return currentStep;
    }
}