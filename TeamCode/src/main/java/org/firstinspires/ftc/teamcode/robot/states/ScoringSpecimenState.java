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
        MOVE_ARM_DOWN_FORWARDS,
        WAIT_FOR_RIGHT_TRIGGER,
        ROTATE_ARM_BACK_TO_HOOK,
        OPEN_CLAW,
        MOVE_ARM_UP,
        LOWER_VIPERLIFT_TO_BOTTOM,
        COMPLETED
    }

    private Step currentStep;
    private static final long OPEN_CLAW_DELAY = 250;

    // Constructor
    public ScoringSpecimenState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.currentStep = Step.LIFT_VIPERLIFT_UP; // Initial step
    }

    // Activate the scoring specimen state
    public void activate() {
        currentStep = Step.LIFT_VIPERLIFT_UP;
        executeCurrentStep();
    }

    // Deactivate the scoring specimen state
    public void deactivate() {
        currentStep = Step.COMPLETED;
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case LIFT_VIPERLIFT_UP:
                // Lift the ViperLift up to a high position
                viperLift.moveToPosition(4000); // Adjust this value as needed
                break;

            case MOVE_ARM_DOWN_FORWARDS:
                // Move the arm down forwards to hang the specimen
                arm.moveToAngle(38);   // Adjust angle as needed
                wrist.setAngle(110);     // Adjust angle as needed
                // Assuming claw is already holding the specimen
                break;

            case WAIT_FOR_RIGHT_TRIGGER:
                // Waiting for right trigger input
                break;

            case ROTATE_ARM_BACK_TO_HOOK:
                // Rotate arm back a predetermined amount to hook the specimen
                arm.moveToAngle(55); // Adjust angle as needed
                // Claw remains closed
                break;

            case OPEN_CLAW:
                // Open the claw to release the specimen
                claw.open();
                waitStartTime = System.currentTimeMillis();
                break;

            case MOVE_ARM_UP:
                // Move arm back up to idle position
                arm.moveToAngle(90);   // Adjust angle as needed
                wrist.setAngle(90);    // Adjust angle as needed
                break;

            case LOWER_VIPERLIFT_TO_BOTTOM:
                // Lower the ViperLift to the bottom position
                viperLift.moveToPosition(0);
                break;

            case COMPLETED:
                // Process completed; deactivate state
                deactivate();
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
                    currentStep = Step.MOVE_ARM_DOWN_FORWARDS;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_DOWN_FORWARDS:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_RIGHT_TRIGGER;
                    // Do not executeCurrentStep() because we're waiting for input
                }
                break;

            case WAIT_FOR_RIGHT_TRIGGER:
                // Waiting for right trigger input
                break;

            case ROTATE_ARM_BACK_TO_HOOK:
                if (arm.isCloseToTarget()) {
                    currentStep = Step.OPEN_CLAW;
                    executeCurrentStep();
                }
                break;

            case OPEN_CLAW:
                if (System.currentTimeMillis() - waitStartTime >= OPEN_CLAW_DELAY) {
                    currentStep = Step.MOVE_ARM_UP;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_UP:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.LOWER_VIPERLIFT_TO_BOTTOM;
                    executeCurrentStep();
                }
                break;

            case LOWER_VIPERLIFT_TO_BOTTOM:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                    executeCurrentStep();
                }
                break;

            case COMPLETED:
                // Process completed; state remains here
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Method to handle right trigger input
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_RIGHT_TRIGGER) {
            currentStep = Step.ROTATE_ARM_BACK_TO_HOOK;
            executeCurrentStep();
        }
    }

    // Get the current step (for telemetry or debugging)
    public Step getCurrentStep() {
        return currentStep;
    }

    // Check if the process is completed
    public boolean isCompleted() {
        return currentStep == Step.COMPLETED;
    }
}
