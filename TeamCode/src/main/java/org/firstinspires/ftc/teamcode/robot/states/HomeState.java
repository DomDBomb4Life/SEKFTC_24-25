// File: HomeState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class HomeState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;

    // Steps in the state machine
    public enum Step {
        MOVE_TO_SAFETY_POSITION,
        WAIT_FOR_INPUT,
        MOVE_TO_PICKUP_POSITION,
        CLOSE_CLAW,
        MOVE_BACK_TO_SAFETY
    }

    private Step currentStep;

    // Safety and pickup positions
    private static final double SAFETY_ARM_ANGLE = 0.0;     // Adjust as needed
    private static final double SAFETY_WRIST_ANGLE = 180.0;  // Adjust as needed
    private static final double PICKUP_ARM_ANGLE = -10.0;   // Adjust as needed
    private static final double PICKUP_WRIST_ANGLE = 160.0;  // Adjust as needed

    // State active flag
    private boolean isActive = false;

    // Constructor
    public HomeState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
    }

    // Activate the home state
    public void start() {
        isActive = true;
        currentStep = Step.MOVE_TO_SAFETY_POSITION;
        executeCurrentStep();
    }

    // Deactivate the home state
    public void deactivate() {
        isActive = false;
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case MOVE_TO_SAFETY_POSITION:
                // Move arm and wrist to safety positions, open claw
                arm.moveToAngle(SAFETY_ARM_ANGLE);
                wrist.setAngle(SAFETY_WRIST_ANGLE);
                claw.open();
                break;

            case WAIT_FOR_INPUT:
                // Do nothing; waiting for right trigger input
                break;

            case MOVE_TO_PICKUP_POSITION:
                // Move arm and wrist to pickup positions
                if (!claw.isOpen()) {
                claw.open();
                executeCurrentStep();
                }
                arm.moveToAngle(PICKUP_ARM_ANGLE);
                wrist.setAngle(PICKUP_WRIST_ANGLE);
                break;

            case CLOSE_CLAW:
                // Close the claw
                claw.close();
                break;

            case MOVE_BACK_TO_SAFETY:
                // Move arm and wrist back to safety positions
                arm.moveToAngle(SAFETY_ARM_ANGLE);
                wrist.setAngle(SAFETY_WRIST_ANGLE);
                break;
        }
    }

    // Update method to be called periodically
    public void update() {
        if (!isActive) return;

        switch (currentStep) {
            case MOVE_TO_SAFETY_POSITION:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_INPUT;
                    // No need to executeCurrentStep() here
                }
                break;

            case WAIT_FOR_INPUT:
                // Waiting for right trigger input
                break;

            case MOVE_TO_PICKUP_POSITION:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.CLOSE_CLAW;
                    executeCurrentStep();
                }
                break;

            case CLOSE_CLAW:
                // Optionally, add a delay here if needed
                currentStep = Step.MOVE_BACK_TO_SAFETY;
                executeCurrentStep();
                break;

            case MOVE_BACK_TO_SAFETY:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_INPUT;
                    // Cycle complete; waiting for next input
                }
                break;
        }
    }

    // Handle right trigger input
    public void onRightTriggerPressed() {
        if (isActive && currentStep == Step.WAIT_FOR_INPUT) {
            currentStep = Step.MOVE_TO_PICKUP_POSITION;
            executeCurrentStep();
        }
    }

    // Check if the state is active
    public boolean isActive() {
        return isActive;
    }

    // Get current step (for telemetry or debugging)
    public Step getCurrentStep() {
        return currentStep;
    }
}