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
        LIFT_UP,
        MOVE_ARM_WRIST,
        WAIT_FOR_TRIGGER_OPEN,  // Wait for right trigger to open claw
        WAIT_FOR_TRIGGER_CLOSE, // Wait for right trigger to close claw
        MOVE_ARM_WRIST_BACK,
        LOWER_LIFT,
        COMPLETED
    }

    private Step currentStep;

    // Constructor
    public ScoringBasketState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.currentStep = Step.LIFT_UP; // Initial step
    }

    // Start the scoring basket process
    public void start() {
        currentStep = Step.LIFT_UP;
        executeCurrentStep();
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case LIFT_UP:
                // Move Viper Lift all the way up
                viperLift.moveToPosition(6780);
                break;

            case MOVE_ARM_WRIST:
                // Move arm and wrist after ViperLift is up
                arm.moveToAngle(148);
                wrist.setAngle(90);
                break;

            case WAIT_FOR_TRIGGER_OPEN:
                // Waiting for right trigger to open the claw
                break;

            case WAIT_FOR_TRIGGER_CLOSE:
                // Waiting for right trigger to close the claw
                break;

            case MOVE_ARM_WRIST_BACK:
                // Move arm and wrist back to idle position
                arm.moveToAngle(90);
                wrist.setAngle(90);
                break;

            case LOWER_LIFT:
                // Move ViperLift back to idle position
                viperLift.moveToPosition(0);
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
            case LIFT_UP:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_WRIST;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_WRIST:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_TRIGGER_OPEN;
                    // No need to executeCurrentStep() since we're waiting for input
                }
                break;

            case WAIT_FOR_TRIGGER_OPEN:
                // Waiting for right trigger input to open the claw
                break;

            case WAIT_FOR_TRIGGER_CLOSE:
                // Waiting for right trigger input to close the claw
                break;

            case MOVE_ARM_WRIST_BACK:
                if (arm.isCloseToTarget()) {
                    currentStep = Step.LOWER_LIFT;
                    executeCurrentStep();
                }
                break;

            case LOWER_LIFT:
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

    // Method to handle right trigger input
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_TRIGGER_OPEN) {
            // Open the claw to drop the object
            claw.open();
            currentStep = Step.WAIT_FOR_TRIGGER_CLOSE;
        } else if (currentStep == Step.WAIT_FOR_TRIGGER_CLOSE) {
            // Close the claw after scoring
            claw.close();
            currentStep = Step.MOVE_ARM_WRIST_BACK;
            executeCurrentStep();
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