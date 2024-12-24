package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class ScoringBasketState extends BaseState {
    public enum Step {
        LIFT_UP,
        MOVE_ARM_WRIST,
        WAIT_TO_OPEN_CLAW,
        OPEN_CLAW,
        WAIT_FOR_CLAW_OPEN,
        MOVE_ARM_WRIST_BACK,
        LOWER_LIFT,
        COMPLETED
    }

    private Step currentStep;
    private long waitStartTime;

    public ScoringBasketState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.LIFT_UP;
        robot.viperLift.moveToPosition(7100);
        robot.arm.moveToAngle(75);
        robot.wrist.setAngle(33);


    }

    @Override
    protected void cleanup() {
        // No special cleanup
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {
            case LIFT_UP:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_WRIST;
                    robot.arm.moveToAngle(123);
                    robot.wrist.setAngle(33);
                }
                break;

            case MOVE_ARM_WRIST:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.WAIT_TO_OPEN_CLAW;
                    if (isAutonomous) {
                        waitStartTime = System.currentTimeMillis();
                    }
                }
                break;

            case WAIT_TO_OPEN_CLAW:
                if (isAutonomous && (System.currentTimeMillis() - waitStartTime >= 250)) {
                    currentStep = Step.OPEN_CLAW;
                }
                // Wait for input in TeleOp or delay in Autonomous
                break;

            case OPEN_CLAW:
                robot.claw.open();
                waitStartTime = System.currentTimeMillis();
                currentStep = Step.WAIT_FOR_CLAW_OPEN;
                break;

            case WAIT_FOR_CLAW_OPEN:
                if (System.currentTimeMillis() - waitStartTime >= 500) {
                    currentStep = Step.MOVE_ARM_WRIST_BACK;
                    robot.arm.moveToAngle(90);
                    robot.wrist.setAngle(90);
                }
                break;

            case MOVE_ARM_WRIST_BACK:
                if (robot.arm.isCloseToTarget()) {
                    currentStep = Step.LOWER_LIFT;
                    robot.viperLift.moveToPosition(0);
                }
                break;

            case LOWER_LIFT:
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
        if (input == UserInput.RIGHT_TRIGGER && currentStep == Step.WAIT_TO_OPEN_CLAW) {
            currentStep = Step.OPEN_CLAW;
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