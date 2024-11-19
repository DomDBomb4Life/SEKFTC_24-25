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
        WAIT_TO_OPEN_CLAW,
        OPEN_CLAW,           // Added this step
        WAIT_FOR_CLAW_OPEN,  // Added this step
        MOVE_ARM_WRIST_BACK,
        LOWER_LIFT,
        COMPLETED
    }

    private Step currentStep;
    private boolean isAutonomous;
    private long waitStartTime;
    private static final long OPEN_CLAW_DELAY = 200; // 1 second delay in milliseconds

    // Constructor
    public ScoringBasketState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this(viperLift, arm, wrist, claw, false);
    }

    public ScoringBasketState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw, boolean isAutonomous) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.isAutonomous = isAutonomous;
        this.currentStep = Step.LIFT_UP; // Initial step
    }

    // Activate the scoring basket state
    public void activate() {
        currentStep = Step.LIFT_UP;
        executeCurrentStep();
    }

    // Deactivate the scoring basket state
    public void deactivate() {
        currentStep = Step.COMPLETED;
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case LIFT_UP:
                // Move Viper Lift all the way up
                viperLift.moveToPosition(8080);
                break;

            case MOVE_ARM_WRIST:
                // Move arm and wrist after ViperLift is up
                arm.moveToAngle(148);
                wrist.setAngle(90);
                break;

            case WAIT_TO_OPEN_CLAW:
                if (isAutonomous) {
                    waitStartTime = System.currentTimeMillis();
                }
                // In TeleOp, wait for user input (handled in onRightTriggerPressed)
                break;

            case OPEN_CLAW:
                // Open the claw to drop the object
                claw.open();
                break;

            case WAIT_FOR_CLAW_OPEN:
                waitStartTime = System.currentTimeMillis();
                // Wait for the claw to finish opening
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
                deactivate();
                break;

            default:
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
                    currentStep = Step.WAIT_TO_OPEN_CLAW;
                    executeCurrentStep();
                }
                break;

            case WAIT_TO_OPEN_CLAW:
                if (isAutonomous) {
                    if (System.currentTimeMillis() - waitStartTime >= OPEN_CLAW_DELAY) {
                        currentStep = Step.OPEN_CLAW;
                        executeCurrentStep();
                    }
                }
                // In TeleOp, wait for user input
                break;

            case OPEN_CLAW:
                if (claw.isOpen()) {
                    currentStep = Step.WAIT_FOR_CLAW_OPEN;
                    executeCurrentStep();
                }
                break;

            case WAIT_FOR_CLAW_OPEN:
                // Wait a moment to ensure the object has dropped
                if (System.currentTimeMillis() - waitStartTime >= OPEN_CLAW_DELAY) {
                    currentStep = Step.MOVE_ARM_WRIST_BACK;
                    executeCurrentStep();
                }
        
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
                    deactivate();
                }
                break;

            case COMPLETED:
                // Process completed
                break;

            default:
                break;
        }
    }

    // Method to handle right trigger input in TeleOp
    public void onRightTriggerPressed() {
        if (!isAutonomous && currentStep == Step.WAIT_TO_OPEN_CLAW) {
            currentStep = Step.OPEN_CLAW;
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