package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class LevelOneAscentState extends BaseState {
    public enum Step {
        MOVE_TO_POSITIONS,
        WAIT_FOR_TRIGGER,
        MOVE_VIPER_AGAIN,
        COMPLETED
    }

    private Step currentStep;

    public LevelOneAscentState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.MOVE_TO_POSITIONS;
        robot.viperLift.moveToPosition(0);
        robot.arm.moveToAngle(30);
        robot.wrist.setAngle(90);
    }

    @Override
    protected void cleanup() {
        // No special cleanup
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {
            case MOVE_TO_POSITIONS:
                if (robot.wrist.isAtTarget() && robot.arm.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_TRIGGER;
                }
                break;

            case WAIT_FOR_TRIGGER:
                // Wait for input in TeleOp
                break;

            case MOVE_VIPER_AGAIN:
                if (robot.arm.isCloseToTarget()) {
                    robot.arm.moveToAngle(55);
                }
                break;

            case COMPLETED:
                // Done
                break;
        }
    }

    @Override
    public void onUserInput(UserInput input) {
        if (!isAutonomous && input == UserInput.RIGHT_TRIGGER ) {
            currentStep = Step.MOVE_VIPER_AGAIN;
            robot.arm.moveToAngle(85);
        }
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