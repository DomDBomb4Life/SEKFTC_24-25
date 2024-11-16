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
        LIFT_VIPERLIFT_UP,
        MOVE_WRIST_AND_CLAW,
        WAIT_FOR_RIGHT_TRIGGER,
        LOWER_VIPERLIFT,
        WAIT_FOR_BUTTON_PRESS,
        OPEN_CLAW,
        MOVE_ARM_AND_WRIST_BACK,
        LOWER_VIPERLIFT_TO_BOTTOM,
        FINAL // Remain here when done
    }

    private Step currentStep;

    // Constructor
    public ScoringSpecimenState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.currentStep = Step.LIFT_VIPERLIFT_UP; // Initial step
    }

    // Start the scoring specimen process
    public void start() {
        currentStep = Step.LIFT_VIPERLIFT_UP;
        executeCurrentStep();
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case LIFT_VIPERLIFT_UP:
                // Lift the ViperLift up to a specific position
                viperLift.moveToPosition(5000); // Adjust this value as needed
                break;

            case MOVE_WRIST_AND_CLAW:
                // Move the wrist and claw to desired positions
                arm.moveToAngle(45);   // Adjust angle as needed
                wrist.setAngle(45); // Adjust angle as needed
                break;

            case WAIT_FOR_RIGHT_TRIGGER:
                // Waiting for right trigger input
                break;

            case LOWER_VIPERLIFT:
                // Lower the ViperLift to a certain position
                viperLift.moveToPosition(4500); // Adjust this value as needed
                break;

            case WAIT_FOR_BUTTON_PRESS:
                // Waiting for primary button or right trigger input
                break;

            case OPEN_CLAW:
                // Open the claw
                claw.open();
                break;

            case MOVE_ARM_AND_WRIST_BACK:
                // Move arm and wrist back to their correct locations
                arm.moveToAngle(90);   // Adjust angle as needed
                wrist.setAngle(90);    // Adjust angle as needed
                break;

            case LOWER_VIPERLIFT_TO_BOTTOM:
                // Lower the ViperLift to the bottom position
                viperLift.moveToPosition(0);
                break;

            case FINAL:
                // Do nothing; remain here
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Update method to be called periodically
    public void update() {
        switch (currentStep) {
            case LIFT_VIPERLIFT_UP:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_WRIST_AND_CLAW;
                    executeCurrentStep();
                }
                break;

            case MOVE_WRIST_AND_CLAW:
                // Assuming wrist movement is immediate; adjust if necessary
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_RIGHT_TRIGGER;
                    executeCurrentStep();
                }
                break;

            case WAIT_FOR_RIGHT_TRIGGER:
                // Waiting for right trigger input
                break;

            case LOWER_VIPERLIFT:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_BUTTON_PRESS;
                }
                break;

            case WAIT_FOR_BUTTON_PRESS:
                // Waiting for primary button or right trigger input
                break;

            case OPEN_CLAW:
                currentStep = Step.MOVE_ARM_AND_WRIST_BACK;
                executeCurrentStep();
                break;

            case MOVE_ARM_AND_WRIST_BACK:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.LOWER_VIPERLIFT_TO_BOTTOM;
                    executeCurrentStep();
                }
                break;

            case LOWER_VIPERLIFT_TO_BOTTOM:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.FINAL;
                }
                break;

            case FINAL:
                // Process completed; remain here
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Method to handle right trigger input
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_RIGHT_TRIGGER) {
            currentStep = Step.LOWER_VIPERLIFT;
            executeCurrentStep();
        } else if (currentStep == Step.WAIT_FOR_BUTTON_PRESS) {
            // Loop back to lift ViperLift up again
            currentStep = Step.LIFT_VIPERLIFT_UP;
            executeCurrentStep();
        }
    }

    // Method to handle primary button input
    public void onPrimaryButtonPressed() {
        if (currentStep == Step.WAIT_FOR_BUTTON_PRESS) {
            currentStep = Step.OPEN_CLAW;
            executeCurrentStep();
        }
    }

    // Get the current step (for telemetry or debugging)
    public Step getCurrentStep() {
        return currentStep;
    }

    // Check if the process is completed
    public boolean isCompleted() {
        return currentStep == Step.FINAL;
    }
}