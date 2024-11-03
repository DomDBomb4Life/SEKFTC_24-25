// File: ScoringSpecimenState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ScoringSpecimenState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;

    // Steps in the scoring specimen process
    public enum Step {
        STEP_1,
        STEP_2,
        LOOP_LOWERING,
        STEP_3,
        COMPLETED
    }

    private Step currentStep;

    // Flag to track loop direction
    private boolean isLowered = false;

    // Constructor
    public ScoringSpecimenState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.currentStep = Step.STEP_1; // Initial step
    }

    // Start the scoring specimen process
    public void start() {
        currentStep = Step.STEP_1;
        executeCurrentStep();
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case STEP_1:
                // Raise Viper Lift to a certain height
                viperLift.moveToPosition(5000); // Adjust this value as needed
                break;

            case STEP_2:
                // Move arm and wrist to set positions
                arm.moveToAngle(90);   // Adjust angle as needed
                wrist.setAngle(45);    // Adjust angle as needed
                break;

            case LOOP_LOWERING:
                // This step will be controlled by user input in the update method
                break;

            case STEP_3:
                // Open the claw and lower Viper Lift all the way down
                claw.open();
                viperLift.moveToPosition(0);
                break;

            case COMPLETED:
                // Process completed
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Update method to be called periodically
    public void update(boolean primaryButtonPressed, boolean secondaryButtonPressed, boolean previousPrimaryButtonState, boolean previousSecondaryButtonState) {
        switch (currentStep) {
            case STEP_1:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.STEP_2;
                    executeCurrentStep();
                }
                break;

            case STEP_2:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.LOOP_LOWERING;
                }
                break;

            case LOOP_LOWERING:
                if (primaryButtonPressed && !previousPrimaryButtonState) {
                    if (isLowered) {
                        // Raise Viper Lift slightly
                        viperLift.moveToPosition(viperLift.getCurrentPosition() + 100); // Adjust increment as needed
                        isLowered = false;
                    } else {
                        // Lower Viper Lift slightly
                        viperLift.moveToPosition(viperLift.getCurrentPosition() - 100); // Adjust decrement as needed
                        isLowered = true;
                    }
                }

                if (secondaryButtonPressed && !previousSecondaryButtonState) {
                    // Proceed to next step
                    currentStep = Step.STEP_3;
                    executeCurrentStep();
                }
                break;

            case STEP_3:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                }
                break;

            case COMPLETED:
                // Do nothing
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

    // Check if the process is completed
    public boolean isCompleted() {
        return currentStep == Step.COMPLETED;
    }
}