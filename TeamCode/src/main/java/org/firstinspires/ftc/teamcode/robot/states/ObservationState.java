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

    // State active flag
    private boolean isActive = false;
    private long waitStartTime = -1; // Default to an invalid timestamp
    private static final long CLOSE_CLAW_DELAY = 250; // 4000 ms delay

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
        // Slightly raise Viper Lift
        viperLift.moveToPosition(500); // Adjust as needed
        // Set arm and wrist positions
        arm.moveToAngle(180);
        wrist.setAngle(90); // Adjust as needed
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
                viperLift.moveToPosition(735); // Adjust as needed
                break;

            case MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW:
                arm.moveToAngle(170); // Adjust angle as needed for backwards position
                wrist.setAngle(90); // Adjust wrist angle as needed
                claw.open();
                break;

            case WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW:
                // Waiting for right trigger input
                break;

            case CLOSE_THE_CLAW:
                // Initialize the waitStartTime if it's not already set
                if (waitStartTime == -1) {
                    waitStartTime = System.currentTimeMillis();
                }
                claw.close(); // Ensure claw is closed during this step
                break;

            case LIFT_VIPERLIFT_TO_SECOND_HEIGHT:
                viperLift.moveToPosition(1330); // Adjust as needed
                break;

            case MOVE_ARM_UP:
                arm.moveToAngle(90); // Adjust angle as needed
                wrist.setAngle(90); // Adjust wrist angle as needed
                break;

            case LOWER_VIPERLIFT:
                viperLift.moveToPosition(0);
                break;

            case COMPLETED:
                deactivate();
                break;

            default:
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
                // Waiting for right trigger input (handled externally)
                break;

            case CLOSE_THE_CLAW:
                // Wait for the delay to elapse
                if (System.currentTimeMillis() - waitStartTime >= CLOSE_CLAW_DELAY) {
                    currentStep = Step.LIFT_VIPERLIFT_TO_SECOND_HEIGHT;
                    waitStartTime = -1; // Reset timer for potential reuse
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
                if (viperLift.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                    executeCurrentStep();
                }
                break;

            case COMPLETED:
                break;

            default:
                break;
        }
    }

    // Method to handle right trigger input
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW) {
            claw.close();
            currentStep = Step.CLOSE_THE_CLAW;
            executeCurrentStep();
        }
    }

    // Check if the state is active
    public boolean isActive() {
        return isActive;
    }
}