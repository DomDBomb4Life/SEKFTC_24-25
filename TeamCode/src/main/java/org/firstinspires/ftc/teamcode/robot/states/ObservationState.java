// File: ObservationState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ObservationState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;

    // Steps in the observation process
    public enum Step {
        LIFT_VIPERLIFT_TO_PRE_HEIGHT,
        MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW,
        WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW,
        CLOSE_THE_CLAW,
        LIFT_VIPERLIFT_TO_SECOND_HEIGHT,
        MOVE_ARM_UP,
        LOWER_VIPERLIFT,
        COMPLETED
    }

    private Step currentStep;
    private boolean isActive = false;
    private static final long OPEN_CLAW_DELAY = 250;

    // Constructor
    public ObservationState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
    }

    // Activate the observation state
    public void activate() {
        isActive = true;
        currentStep = Step.LIFT_VIPERLIFT_TO_PRE_HEIGHT;
        executeCurrentStep();
    }

    // Deactivate the observation state
    public void deactivate() {
        isActive = false;
        currentStep = Step.COMPLETED;
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case LIFT_VIPERLIFT_TO_PRE_HEIGHT:
                // Raise ViperLift to predetermined height
                viperLift.moveToPosition(670); // Adjust as needed
                break;

            case MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW:
                // Move arm down backwards and open claw
                arm.moveToAngle(178); // Adjust angle as needed for backwards position
                wrist.setAngle(90); // Adjust wrist angle as needed
                claw.open();
                break;

            case WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW:
                // Waiting for right trigger input
                break;

            case CLOSE_THE_CLAW:
                //i need the claw closed AAAAA
                claw.close();
                waitStartTime = System.currentTimeMillis();
                break;

            case LIFT_VIPERLIFT_TO_SECOND_HEIGHT:
                // Lift ViperLift up more to take specimen off the wall
                viperLift.moveToPosition(950); // Adjust as needed
                break;

            case MOVE_ARM_UP:
                // Move arm back up to idle position
                arm.moveToAngle(90); // Adjust angle as needed
                wrist.setAngle(90); // Adjust wrist angle as needed
                break;

            case LOWER_VIPERLIFT:
                // Lower ViperLift to the bottom position
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

    // Update method
    public void update() {
        if (!isActive) return;

        switch (currentStep) {
            case LIFT_VIPERLIFT_TO_PRE_HEIGHT:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW;
                }
                break;

            case WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW:
                // Waiting for right trigger input
                break;

            case CLOSE_THE_CLAW:
                if (System.currentTimeMillis() - waitStartTime >= OPEN_CLAW_DELAY) {
                    currentStep = Step.LIFT_VIPERLIFT_TO_SECOND_HEIGHT;
                    executeCurrentStep();
                }
                break;

            case LIFT_VIPERLIFT_TO_SECOND_HEIGHT:
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_UP;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_UP:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.LOWER_VIPERLIFT;
                    executeCurrentStep();
                }
                break;

            case LOWER_VIPERLIFT:
                if (arm.getCurrentAngle() < 145) {
                    currentStep = Step.COMPLETED;
                    executeCurrentStep();
                }
                break;

            case COMPLETED:
                // Do nothing; state remains here
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Method to handle right trigger input
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW) {
            // Close claw to grab specimen
            claw.close();
            currentStep = Step.LIFT_VIPERLIFT_TO_SECOND_HEIGHT;
            executeCurrentStep();
        }
    }

    // Check if the state is active
    public boolean isActive() {
        return isActive;
    }

    // For telemetry or debugging
    public Step getCurrentStep() {
        return currentStep;
    }
}
