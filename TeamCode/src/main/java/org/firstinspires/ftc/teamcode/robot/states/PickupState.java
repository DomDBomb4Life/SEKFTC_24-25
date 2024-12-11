package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class PickupState extends BaseState {
    public enum Step {
        MOVE_TO_PICKUP_POSITION,
        WAIT_FOR_DRIVE_FORWARD,
        CLOSE_CLAW,
        LIFT_UP,
        COMPLETED
    }

    private Step currentStep;
    private long waitStartTime;

    public PickupState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.MOVE_TO_PICKUP_POSITION;
        robot.viperLift.moveToPosition(0);
        robot.arm.moveToAngle(-20);
        robot.wrist.setAngle(75);
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
            case MOVE_TO_PICKUP_POSITION:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget() && robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_DRIVE_FORWARD;
                }
                break;

            case WAIT_FOR_DRIVE_FORWARD:
                // Wait for user input in TeleOp or external signal in Autonomous
                break;

            case CLOSE_CLAW:
                if (System.currentTimeMillis() - waitStartTime >= 200) {
                    robot.claw.close();
                    currentStep = Step.LIFT_UP;
                }
                break;

            case LIFT_UP:
                if (robot.arm.isCloseToTarget()) {
                    robot.arm.moveToAngle(90);
                    robot.viperLift.moveToPosition(0);
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
        if (input == UserInput.RIGHT_TRIGGER && currentStep == Step.WAIT_FOR_DRIVE_FORWARD) {
            currentStep = Step.CLOSE_CLAW;
            waitStartTime = System.currentTimeMillis();
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