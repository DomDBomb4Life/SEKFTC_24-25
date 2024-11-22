// File: LevelOneAscentState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class LevelOneAscentState extends BaseState {
    // Steps in the ascent process
    public enum Step {
        MOVE_TO_POSITIONS,    // Step 1
        WAIT_FOR_TRIGGER,     // Step 2 (skipped in Autonomous)
        MOVE_VIPER_AGAIN,     // Step 3
        FINAL                 // Step 4
    }

    private Step currentStep;

    // Constructor
    public LevelOneAscentState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.MOVE_TO_POSITIONS;
        executeCurrentStep();
    }

    private void executeCurrentStep() {
        switch (currentStep) {
            case MOVE_TO_POSITIONS:
                // Move ViperLift and Arm to desired positions
                robot.viperLift.moveToPosition(5000); // Adjust this value as needed
                robot.arm.moveToAngle(0);            // Adjust this angle as needed
                break;

            case WAIT_FOR_TRIGGER:
                if (isAutonomous) {
                    // Skip waiting for trigger in Autonomous
                    currentStep = Step.MOVE_VIPER_AGAIN;
                    executeCurrentStep();
                }
                // In TeleOp, wait for user input
                break;

            case MOVE_VIPER_AGAIN:
                // Move ViperLift again
                robot.viperLift.moveToPosition(3930); // Adjust this value as needed
                break;

            case FINAL:
                // Do nothing; state remains here
                break;

            default:
                break;
        }
    }

    @Override
    public void update() {
        if (!isActive) return;

        switch (currentStep) {
            case MOVE_TO_POSITIONS:
                if (robot.viperLift.isCloseToTarget() && robot.arm.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_TRIGGER;
                    executeCurrentStep();
                }
                break;

            case WAIT_FOR_TRIGGER:
                // In Autonomous, this step is skipped
                break;

            case MOVE_VIPER_AGAIN:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.FINAL;
                    deactivate();
                }
                break;

            case FINAL:
                // Do nothing; state remains here
                break;

            default:
                break;
        }
    }

    @Override
    public void onRightTriggerPressed() {
        if (!isAutonomous && currentStep == Step.WAIT_FOR_TRIGGER) {
            currentStep = Step.MOVE_VIPER_AGAIN;
            executeCurrentStep();
        }
    }

    @Override
    public boolean isCompleted() {
        return currentStep == Step.FINAL;
    }

    @Override
    public String getCurrentStep() {
        return currentStep.toString();
    }
}