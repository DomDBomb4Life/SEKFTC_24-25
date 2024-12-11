package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class ScoringSpecimenState extends BaseState {
    public enum Step {
        LIFT_VIPERLIFT_UP,
        MOVE_ARM_DOWN_FORWARDS,
        WAIT_FOR_RIGHT_TRIGGER,
        MOVE_VIPER_LIFT_UP_TO_HOOK,
        OPEN_CLAW,
        MOVE_ARM_UP,
        LOWER_VIPERLIFT_TO_BOTTOM,
        COMPLETED
    }

    private Step currentStep;
    private long waitStartTime;

    public ScoringSpecimenState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.LIFT_VIPERLIFT_UP;
        robot.viperLift.moveToPosition(1987);
    }

    @Override
    protected void cleanup() {
        // No special cleanup
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {
            case LIFT_VIPERLIFT_UP:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_DOWN_FORWARDS;
                    robot.arm.moveToAngle(38);
                    robot.wrist.setAngle(110);
                }
                break;

            case MOVE_ARM_DOWN_FORWARDS:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_RIGHT_TRIGGER;
                }
                break;

            case WAIT_FOR_RIGHT_TRIGGER:
                // Wait for input
                break;

            case MOVE_VIPER_LIFT_UP_TO_HOOK:
                robot.viperLift.moveToPosition(4000);
                robot.arm.moveToAngle(105);
                currentStep = Step.OPEN_CLAW;
                break;

            case OPEN_CLAW:
                robot.claw.open();
                waitStartTime = System.currentTimeMillis();
                currentStep = Step.MOVE_ARM_UP;
                break;

            case MOVE_ARM_UP:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.LOWER_VIPERLIFT_TO_BOTTOM;
                    robot.viperLift.moveToPosition(0);
                }
                break;

            case LOWER_VIPERLIFT_TO_BOTTOM:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                }
                break;

            case COMPLETED:
                // Done
                break;
        }
    }

    @Override
    public void onUserInput(UserInput input) {
        if (input == UserInput.RIGHT_TRIGGER && currentStep == Step.WAIT_FOR_RIGHT_TRIGGER) {
            currentStep = Step.MOVE_VIPER_LIFT_UP_TO_HOOK;
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