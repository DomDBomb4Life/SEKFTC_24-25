package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class InitializeArmState extends BaseState {
    public enum Step {
        KILL_ARM_POWER,
        INIT_ARM,
        COMPLETED
    }

    private Step currentStep;

    private static final long DELAY = 5000;
    private long waitStartTime;

    public InitializeArmState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        robot.arm.moveToAngle(110); // Move arm to idle position
        currentStep = Step.KILL_ARM_POWER;
    }

    @Override
    protected void cleanup() {
        // No specific cleanup needed for this state
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {
            case KILL_ARM_POWER:
                if (robot.arm.isCloseToTarget()) {
                    robot.arm.stop();
                    currentStep = Step.INIT_ARM;
                    waitStartTime = System.currentTimeMillis();
                }
                break;

            case INIT_ARM:
                if (System.currentTimeMillis() - waitStartTime >= DELAY) {
                    robot.arm.initializeEncoder(137);
                    currentStep = Step.COMPLETED;
                }
                break;

            case COMPLETED:
                // Final state; no further actions needed
                break;
        }
    }

    @Override
    public void onUserInput(UserInput input) {
        // This state does not handle user input directly
    }

    @Override
    public boolean isCompleted() {
        return currentStep == Step.COMPLETED;
    }

    @Override
    public String getCurrentStep() {
        return currentStep.toString();
    }
}