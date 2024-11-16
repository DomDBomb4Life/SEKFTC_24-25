// File: LevelOneAscentState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * State for achieving a Level One Ascent.
 */
public class LevelOneAscentState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;

    // Steps in the ascent process
    public enum Step {
        MOVE_TO_POSITIONS,    // Step 1
        WAIT_FOR_TRIGGER,     // Step 2
        MOVE_VIPER_AGAIN,     // Step 3
        FINAL                 // Step 4
    }

    private Step currentStep;

    // Constructor
    public LevelOneAscentState(ViperLift viperLift, Arm arm) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.currentStep = Step.MOVE_TO_POSITIONS; // Initial step
    }

    // Start the ascent process
    public void start() {
        currentStep = Step.MOVE_TO_POSITIONS;
        executeCurrentStep();
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case MOVE_TO_POSITIONS:
                // Move ViperLift and Arm to desired positions
                viperLift.moveToPosition(5000); // Adjust this value as needed
                arm.moveToAngle(0);            // Adjust this angle as needed
                break;

            case WAIT_FOR_TRIGGER:
                // Waiting for right trigger input
                break;

            case MOVE_VIPER_AGAIN:
                // Move ViperLift again
                viperLift.moveToPosition(3930); // Adjust this value as needed
                break;

            case FINAL:
                // Do nothing; state remains here
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Update method to be called periodically
    public void update() {
        switch (currentStep) {
            case MOVE_TO_POSITIONS:
                if (viperLift.isCloseToTarget() && arm.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_TRIGGER;
                    // Do not execute next step yet
                }
                break;

            case WAIT_FOR_TRIGGER:
                // Do nothing; waiting for right trigger input
                break;

            case MOVE_VIPER_AGAIN:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.FINAL;
                    // Do not move to COMPLETED; remain in FINAL step
                }
                break;

            case FINAL:
                // Do nothing; state remains here
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Method to handle right trigger input
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_TRIGGER) {
            currentStep = Step.MOVE_VIPER_AGAIN;
            executeCurrentStep();
        }
    }

    // Get the current step (for telemetry or debugging)
    public Step getCurrentStep() {
        return currentStep;
    }
}