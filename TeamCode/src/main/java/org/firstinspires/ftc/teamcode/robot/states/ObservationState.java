package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class ObservationState extends BaseState {
    public enum Step {
        LIFT_VIPERLIFT_TO_PRE_HEIGHT,
        MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW,
        WAIT_FOR_TRIGGER,
        CLOSE_CLAW,
        WAIT_CLAW_CLOSE_DELAY,
        LIFT_VIPERLIFT_TO_SECOND_HEIGHT,
        MOVE_ARM_UP,
        LOWER_VIPERLIFT,
        COMPLETED
    }

    private Step currentStep;
    private static final long OPEN_CLAW_DELAY = 250;
    private long waitStartTime;

    public ObservationState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.LIFT_VIPERLIFT_TO_PRE_HEIGHT;
        robot.viperLift.moveToPosition(470);
        robot.wrist.setAngle(99);
        robot.arm.moveToAngle(168);
        robot.claw.open();
    }

    @Override
    protected void cleanup() {
        // No special cleanup
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {
            case LIFT_VIPERLIFT_TO_PRE_HEIGHT:
                if (robot.viperLift.isCloseToTarget()  &&  robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_TRIGGER;

                }
                break;


            case WAIT_FOR_TRIGGER:
                // Wait for user input (RIGHT_TRIGGER)
                break;

            case CLOSE_CLAW:
                robot.claw.close();
                waitStartTime = System.currentTimeMillis();
                currentStep = Step.WAIT_CLAW_CLOSE_DELAY;
                break;

            case WAIT_CLAW_CLOSE_DELAY:
                if (System.currentTimeMillis() - waitStartTime >= OPEN_CLAW_DELAY) {
                    currentStep = Step.LIFT_VIPERLIFT_TO_SECOND_HEIGHT;
                    robot.viperLift.moveToPosition(731);
                }
                break;

            case LIFT_VIPERLIFT_TO_SECOND_HEIGHT:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_UP;
                    robot.arm.moveToAngle(90);
                    robot.wrist.setAngle(90);

                }
                break;

            case MOVE_ARM_UP:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.LOWER_VIPERLIFT;
                    robot.viperLift.moveToPosition(0);
                }
                break;

            case LOWER_VIPERLIFT:
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
        if (input == UserInput.RIGHT_TRIGGER && currentStep == Step.WAIT_FOR_TRIGGER) {
            currentStep = Step.CLOSE_CLAW;
            robot.claw.close();
            waitStartTime = System.currentTimeMillis();
            currentStep = Step.WAIT_CLAW_CLOSE_DELAY;
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