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
        OPEN_CLAW_BEFORE_PICKUP,
        WAIT_AFTER_OPENING_CLAW,
        MOVE_WRIST_TO_PICKUP_POSITION,
        MOVE_ARM_TO_PICKUP_POSITION,
        CLOSE_CLAW,
        WAIT_AFTER_CLOSING_CLAW,
        MOVE_BACK_TO_SAFETY
    }

    private Step currentStep;

    // Safety and pickup positions
    private static final double SAFETY_ARM_ANGLE = 0.0;     // Adjust as needed
    private static final double SAFETY_WRIST_ANGLE = 125.0; // Adjust as needed
    private static final double PICKUP_ARM_ANGLE = -14.0;   // Adjust as needed
    private static final double PICKUP_WRIST_ANGLE = 125.0; // Adjust as needed

    // Timer variables
    private long stepStartTime;
    private static final long WAIT_DURATION_MS = 100; // 1-second wait

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

            case OPEN_CLAW_BEFORE_PICKUP:
                // Ensure the claw is open
                claw.open();
                stepStartTime = System.currentTimeMillis(); // Start wait timer
                break;

            case WAIT_AFTER_OPENING_CLAW:
                // Do nothing; waiting for the timer to complete
                break;

            case MOVE_WRIST_TO_PICKUP_POSITION:
                // Move wrist to pickup position
                wrist.setAngle(PICKUP_WRIST_ANGLE);
                break;

            case MOVE_ARM_TO_PICKUP_POSITION:
                // Move arm to pickup position
                arm.moveToAngle(PICKUP_ARM_ANGLE);
                break;

            case CLOSE_CLAW:
                // Close the claw
                claw.close();
                stepStartTime = System.currentTimeMillis(); // Start wait timer
                break;

            case WAIT_AFTER_CLOSING_CLAW:
                // Do nothing; waiting for the timer to complete
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
                }
                break;

            case WAIT_FOR_INPUT:
                // Waiting for right trigger input
                break;

            case OPEN_CLAW_BEFORE_PICKUP:
                if (claw.isOpen()) {
                    currentStep = Step.WAIT_AFTER_OPENING_CLAW;
                }
                break;

            case WAIT_AFTER_OPENING_CLAW:
                long elapsedTimeAfterOpen = System.currentTimeMillis() - stepStartTime;
                if (elapsedTimeAfterOpen >= WAIT_DURATION_MS) {
                    currentStep = Step.MOVE_WRIST_TO_PICKUP_POSITION;
                    executeCurrentStep();
                }
                break;

            case MOVE_WRIST_TO_PICKUP_POSITION:
                if (wrist.isAtTarget()) {
                    currentStep = Step.MOVE_ARM_TO_PICKUP_POSITION;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_TO_PICKUP_POSITION:
                if (arm.isCloseToTarget()) {
                    currentStep = Step.CLOSE_CLAW;
                    executeCurrentStep();
                }
                break;

            case CLOSE_CLAW:
                currentStep = Step.WAIT_AFTER_CLOSING_CLAW;
                break;

            case WAIT_AFTER_CLOSING_CLAW:
                long elapsedTimeAfterClose = System.currentTimeMillis() - stepStartTime;
                if (elapsedTimeAfterClose >= WAIT_DURATION_MS) {
                    currentStep = Step.MOVE_BACK_TO_SAFETY;
                    executeCurrentStep();
                }
                break;

            case MOVE_BACK_TO_SAFETY:
                if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_INPUT;
                }
                break;
        }
    }

    // Handle right trigger input
    public void onRightTriggerPressed() {
        if (isActive && currentStep == Step.WAIT_FOR_INPUT) {
            currentStep = Step.OPEN_CLAW_BEFORE_PICKUP;
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