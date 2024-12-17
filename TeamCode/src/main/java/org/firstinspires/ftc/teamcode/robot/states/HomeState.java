package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class HomeState extends BaseState {
    public enum Step {
        MOVE_TO_SAFETY_POSITION,
        WAIT_FOR_INPUT,
        OPEN_CLAW_BEFORE_PICKUP,
        WAIT_AFTER_OPENING_CLAW,
        MOVE_WRIST_TO_PICKUP_POSITION,
        MOVE_ARM_TO_PICKUP_POSITION,
        CLOSE_CLAW,
        WAIT_AFTER_CLOSING_CLAW,
        MOVE_BACK_TO_SAFETY,
        COMPLETED
    }

    private Step currentStep;
    private static final double SAFETY_ARM_ANGLE = 0.0;
    private static final double SAFETY_WRIST_ANGLE = 125.0;
    private static final double PICKUP_ARM_ANGLE = -14.0;
    private static final double PICKUP_WRIST_ANGLE = 125.0;
    private long stepStartTime;
    private static final long WAIT_DURATION_MS = 100;
    private static final long ARM_WAIT_DURATION_MS = 750;


    private boolean useMediumClose = false; // Added: to track which close method to use

    public HomeState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.MOVE_TO_SAFETY_POSITION;
        robot.arm.moveToAngle(0);
        robot.wrist.setAngle(146);
        robot.claw.open();
    }

    @Override
    protected void cleanup() {
        // Nothing special to cleanup
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {
            case MOVE_TO_SAFETY_POSITION:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_INPUT;
                }
                break;

            case WAIT_FOR_INPUT:
                // Wait for user input
                break;

            case OPEN_CLAW_BEFORE_PICKUP:
                if (robot.claw.isOpen()) {
                    currentStep = Step.WAIT_AFTER_OPENING_CLAW;
                    stepStartTime = System.currentTimeMillis();
                }
                break;

            case WAIT_AFTER_OPENING_CLAW:
                if (System.currentTimeMillis() - stepStartTime >= WAIT_DURATION_MS) {
                    currentStep = Step.MOVE_WRIST_TO_PICKUP_POSITION;
                    robot.wrist.setAngle(SAFETY_WRIST_ANGLE);
                }
                break;

            case MOVE_WRIST_TO_PICKUP_POSITION:
                if (robot.wrist.isAtTarget()) {
                    currentStep = Step.MOVE_ARM_TO_PICKUP_POSITION;
                    robot.arm.moveToAngle(PICKUP_ARM_ANGLE);
                    stepStartTime = System.currentTimeMillis();
                }
                break;

            case MOVE_ARM_TO_PICKUP_POSITION:
                if (System.currentTimeMillis()-stepStartTime >= ARM_WAIT_DURATION_MS) {
                    currentStep = Step.CLOSE_CLAW;
                    // Here we close based on useMediumClose
                    if (useMediumClose) {
                        robot.claw.closeMedium();
                    } else {
                        robot.claw.close();
                    }
                    stepStartTime = System.currentTimeMillis();
                }
                break;

            case CLOSE_CLAW:
                currentStep = Step.WAIT_AFTER_CLOSING_CLAW;
                break;

            case WAIT_AFTER_CLOSING_CLAW:
                if (System.currentTimeMillis() - stepStartTime >= WAIT_DURATION_MS) {
                    currentStep = Step.MOVE_BACK_TO_SAFETY;
                    robot.arm.moveToAngle(SAFETY_ARM_ANGLE);
                    robot.wrist.setAngle(SAFETY_WRIST_ANGLE);
                }
                break;

            case MOVE_BACK_TO_SAFETY:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_INPUT;
                }
                break;

            case COMPLETED:
                // Done
                break;
        }
    }

    @Override
    public void onUserInput(UserInput input) {
        // If right trigger pressed at WAIT_FOR_INPUT: normal flow
        // If left trigger pressed at WAIT_FOR_INPUT: medium close flow
        if (currentStep == Step.WAIT_FOR_INPUT) {
            if (input == UserInput.RIGHT_TRIGGER) {
                useMediumClose = false;
                currentStep = Step.OPEN_CLAW_BEFORE_PICKUP;
                robot.claw.open();
            } else if (input == UserInput.PRIMARY_BUTTON) {
                useMediumClose = true;
                currentStep = Step.OPEN_CLAW_BEFORE_PICKUP;
                robot.claw.open();
            }
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