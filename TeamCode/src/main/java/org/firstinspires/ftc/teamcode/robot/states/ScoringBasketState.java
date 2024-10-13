// File: ScoringBasketState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ScoringBasketState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;

    // Steps in the scoring basket process
    public enum Step {
        STEP_1,
        STEP_2,
        STEP_3,
        STEP_4,
        STEP_5,
        COMPLETED
    }

    private Step currentStep;

    // Timer for step 3
    private long stepStartTime;
    private static final long STEP_3_DURATION_MS = 1000; // Adjust the duration as needed

    // Constructor
    public ScoringBasketState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.currentStep = Step.STEP_1; // Initial step
    }

    // Start the scoring basket process
    public void start() {
        currentStep = Step.STEP_1;
        executeCurrentStep();
    }

    // Progress to the next step based on button presses
    public void progress() {
        switch (currentStep) {
            case STEP_1:
                // No progression needed; wait for automatic step 2
                break;
            case STEP_2:
                // User needs to press button to open claw (step 3)
                currentStep = Step.STEP_3;
                executeCurrentStep();
                break;
            case STEP_3:
                // No progression needed; wait for automatic step 4
                break;
            default:
                // Already completed or invalid step
                break;
        }
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case STEP_1:
                // Move Viper Lift all the way up
                viperLift.moveToMax();
                break;

            case STEP_2:
                // Automatically move arm and wrist after ViperLift is up
                arm.moveToAngle(180);
                wrist.setAngle(180);
                break;

            case STEP_3:
                // Open the claw to drop the object
                claw.open();
                // Start timer
                stepStartTime = System.currentTimeMillis();
                break;

            case STEP_4:
                // Automatically move arm and wrist back to idle position
                arm.moveToAngle(90);
                wrist.setAngle(90);
                break;

            case STEP_5:
                // Automatically move ViperLift back to idle position
                viperLift.moveToPosition(0);
                break;

            case COMPLETED:
                // Scoring process completed
                // Optionally transition to idle state
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Update method to be called periodically
    public void update() {
        boolean actionCompleted = false;

        switch (currentStep) {
            case STEP_1:
                if (viperLift.isAtTarget()) {
                    // Proceed to automatic step 2
                    currentStep = Step.STEP_2;
                    executeCurrentStep();
                }
                break;

            case STEP_2:
                if (arm.isAtTarget() && wrist.isAtTarget()) {
                    // Wait for user input to proceed to step 3
                    // User should press the button to progress()
                }
                break;

            case STEP_3:
                // Wait for timer to complete
                long elapsedTime = System.currentTimeMillis() - stepStartTime;
                if (elapsedTime >= STEP_3_DURATION_MS) {
                    // Proceed to automatic step 4
                    currentStep = Step.STEP_4;
                    executeCurrentStep();
                }
                break;

            case STEP_4:
                if (arm.isAtTarget() && wrist.isAtTarget()) {
                    // Proceed to automatic step 5
                    currentStep = Step.STEP_5;
                    executeCurrentStep();
                }
                break;

            case STEP_5:
                if (viperLift.isAtTarget()) {
                    // Process completed
                    currentStep = Step.COMPLETED;
                    // Optionally transition to idle state
                }
                break;

            case COMPLETED:
                // No action needed
                actionCompleted = true;
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Get the current step
    public Step getCurrentStep() {
        return currentStep;
    }
}