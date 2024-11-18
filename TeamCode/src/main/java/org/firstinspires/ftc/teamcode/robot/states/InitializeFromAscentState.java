// File: InitializeFromAscentState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class InitializeFromAscentState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;

    // Steps in the initialization process
    public enum Step {
        INIT_VIPER_LIFT,
        LIFT_UP,
        INIT_ARM,
        MOVE_ARM_TO_ZERO,
        WAIT_FOR_TRIGGER,
        MOVE_ARM_TO_IDLE,
        COMPLETED
    }

    private Step currentStep;
    private boolean isActive = false;

    // Constructor
    public InitializeFromAscentState(ViperLift viperLift, Arm arm) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.currentStep = Step.INIT_VIPER_LIFT;
    }

    // Activate the initialization state
    public void activate() {
        isActive = true;
        currentStep = Step.INIT_VIPER_LIFT;
        executeCurrentStep();
    }

    // Deactivate the state
    public void deactivate() {
        isActive = false;
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case INIT_VIPER_LIFT:
                // Initialize ViperLift position to 3930 (current physical position)
                viperLift.initializePosition(3930);
                break;

            case LIFT_UP:
                // Move ViperLift up to 5000
                viperLift.moveToPosition(5000);
                // Cut power to the arm to make it go limp
                arm.setPower(0.0);
                break;

            case INIT_ARM:
                // Initialize the arm encoder at -18 degrees
                arm.initializeEncoder();
                break;

            case MOVE_ARM_TO_ZERO:
                // Move arm to 0 degrees
                arm.moveToAngle(0);
                break;

            case WAIT_FOR_TRIGGER:
                // Waiting for driver input (right trigger)
                break;

            case MOVE_ARM_TO_IDLE:
                // Move arm to 90 degrees (idle position)
                arm.moveToAngle(90);
                viperLift.moveToPosition(0);
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
            case INIT_VIPER_LIFT:
                currentStep = Step.LIFT_UP;
                executeCurrentStep();
                break;

            case LIFT_UP:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.INIT_ARM;
                    executeCurrentStep();
                }
                break;

            case INIT_ARM:
                // Small delay can be added if needed
                currentStep = Step.MOVE_ARM_TO_ZERO;
                executeCurrentStep();
                break;

            case MOVE_ARM_TO_ZERO:
                if (arm.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_TRIGGER;
                }
                break;

            case WAIT_FOR_TRIGGER:
                // Waiting for right trigger input
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

    // Method to handle right trigger input
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_TRIGGER) {
            currentStep = Step.MOVE_ARM_TO_IDLE;
            executeCurrentStep();
        }
    }

    // Get current step for telemetry
    public Step getCurrentStep() {
        return currentStep;
    }
}