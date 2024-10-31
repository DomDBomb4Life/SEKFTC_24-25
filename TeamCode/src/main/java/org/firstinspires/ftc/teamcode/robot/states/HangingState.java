// File: HangingState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class HangingState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;

    // Steps in the hanging process
    public enum Step {
        STEP_1,
        STEP_2,
        STEP_3,
        STEP_4,
        COMPLETED
    }

    private Step currentStep;

    // Constructor
    public HangingState(ViperLift viperLift, Arm arm) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.currentStep = Step.STEP_1; // Initial step
    }

    // Start the hanging process
    public void start() {
        currentStep = Step.STEP_1;
        executeCurrentStep();
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case STEP_1:
                // Move Viper Lift up slightly
                viperLift.moveToPosition(500); // Adjust the position value as needed
                break;

            case STEP_2:
                // Extend arm to 0 degrees
                arm.moveToAngle(0);
                break;

            case STEP_3:
                // Raise Viper Lift to maximum height
                viperLift.moveToMax();
                break;

            case STEP_4:
                // Adjust arm to approximately 80 degrees
                arm.moveToAngle(80);
                break;

            case COMPLETED:
                // Hanging process completed
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
                if (arm.isCloseToTarget()) {
                    currentStep = Step.STEP_3;
                    executeCurrentStep();
                }
                break;

            case STEP_3:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.STEP_4;
                    executeCurrentStep();
                }
                break;

            case STEP_4:
                if (arm.isCloseToTarget()) {
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
}