// File: InitializeFromFloorState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class InitializeFromFloorState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;

    // Steps in the initialization process
    public enum Step {
        INIT_ARM,
        MOVE_ARM_TO_IDLE,
        COMPLETED
    }

    private Step currentStep;
    private boolean isActive = false;

    // Constructor
    public InitializeFromFloorState(ViperLift viperLift, Arm arm) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.currentStep = Step.INIT_ARM;
    }

    // Activate the initialization state
    public void activate() {
        isActive = true;
        currentStep = Step.INIT_ARM;
        executeCurrentStep();
    }

    // Deactivate the state
    public void deactivate() {
        isActive = false;
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case INIT_ARM:
                // Initialize the arm encoder at -18 degrees
                arm.initializeEncoder();
                break;

            case MOVE_ARM_TO_IDLE:
                // Move arm to 90 degrees (idle position)
                arm.moveToAngle(90);
                break;

            case COMPLETED:
                // Initialization complete
                break;

            default:
                break;
        }
    }

    // Update method to be called periodically
    public void update() {
        if (!isActive) return;

        switch (currentStep) {
            case INIT_ARM:
                // Small delay can be added if needed
                currentStep = Step.MOVE_ARM_TO_IDLE;
                executeCurrentStep();
                break;

            case MOVE_ARM_TO_IDLE:
                if (arm.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                    deactivate();
                }
                break;

            case COMPLETED:
                // Do nothing
                break;

            default:
                break;
        }
    }

    // Get current step for telemetry
    public Step getCurrentStep() {
        return currentStep;
    }
}