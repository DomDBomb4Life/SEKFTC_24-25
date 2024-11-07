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

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case STEP_1:
                // Move Viper Lift all the way up
                viperLift.moveToPosition(9286);
                break;

            case STEP_2:
                // Automatically move arm and wrist after ViperLift is up
                arm.moveToAngle(120);
                wrist.setAngle(150);
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
                claw.close();
                break;

            case COMPLETED:
                // Scoring process completed
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Update method to be called periodically
    public void update() {
        switch (currentStep) {
            case STEP_1:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.STEP_2;
                    executeCurrentStep();
                }
                break;

            case STEP_2:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.STEP_3;
                    executeCurrentStep();
                }
                break;

            case STEP_3:
                // Wait for timer to complete
                long elapsedTime = System.currentTimeMillis() - stepStartTime;
                if (elapsedTime >= STEP_3_DURATION_MS) {
                    currentStep = Step.STEP_4;
                    executeCurrentStep();
                }
                break;

            case STEP_4:
                if (arm.isCloseToTarget() ) {
                    currentStep = Step.STEP_5;
                    executeCurrentStep();
                }
                break;

            case STEP_5:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                }
                break;

            case COMPLETED:
                // Process completed
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

    // Check if the scoring process is completed
    public boolean isCompleted() {
        return currentStep == Step.COMPLETED;
    }
}