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
        WAIT_FOR_TRIGGER,     // Step 2 (skipped in Autonomous)
        MOVE_VIPER_AGAIN,     // Step 3
        FINAL                 // Step 4
    }

    private Step currentStep;
    private boolean isAutonomous;

    // Constructor
    public LevelOneAscentState(ViperLift viperLift, Arm arm) {
        this(viperLift, arm, false);
    }

    public LevelOneAscentState(ViperLift viperLift, Arm arm, boolean isAutonomous) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.isAutonomous = isAutonomous;
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
                if (isAutonomous) {
                    // Skip waiting for trigger in Autonomous
                    currentStep = Step.MOVE_VIPER_AGAIN;
                    executeCurrentStep();
                }
                // In TeleOp, wait for user input
                break;

            case MOVE_VIPER_AGAIN:
                // Move ViperLift again
                viperLift.moveToPosition(3930); // Adjust this value as needed
                break;

            case FINAL:
                // Do nothing; state remains here
                break;

            default:
                break;
        }
    }

    // Update method to be called periodically
    public void update() {
        switch (currentStep) {
            case MOVE_TO_POSITIONS:
                if (viperLift.isCloseToTarget() && arm.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_TRIGGER;
                    executeCurrentStep();
                }
                break;

            case WAIT_FOR_TRIGGER:
                // In Autonomous, this step is skipped
                break;

            case MOVE_VIPER_AGAIN:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.FINAL;
                }
                break;

            case FINAL:
                // Do nothing; state remains here
                break;

            default:
                break;
        }
    }

    // Method to handle right trigger input
    public void onRightTriggerPressed() {
        if (!isAutonomous && currentStep == Step.WAIT_FOR_TRIGGER) {
            currentStep = Step.MOVE_VIPER_AGAIN;
            executeCurrentStep();
        }
    }

    // Get the current step (for telemetry or debugging)
    public Step getCurrentStep() {
        return currentStep;
    }

    // Check if the ascent process is completed

}