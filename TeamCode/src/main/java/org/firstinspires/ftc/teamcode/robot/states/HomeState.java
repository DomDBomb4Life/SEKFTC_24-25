// File: HomeState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class HomeState extends BaseState {
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
    private static final double SAFETY_ARM_ANGLE = 0.0;
    private static final double SAFETY_WRIST_ANGLE = 125.0;
    private static final double PICKUP_ARM_ANGLE = -14.0;
    private static final double PICKUP_WRIST_ANGLE = 125.0;
    private long stepStartTime;
    private static final long WAIT_DURATION_MS = 100; // Wait duration in milliseconds

    // Constructor
    public HomeState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.MOVE_TO_SAFETY_POSITION;
        executeCurrentStep();
    }

    @Override
    public void deactivate() {
        super.deactivate();
    }

    private void executeCurrentStep() {
        switch (currentStep) {
            case MOVE_TO_SAFETY_POSITION:
                robot.arm.moveToAngle(SAFETY_ARM_ANGLE);
                robot.wrist.setAngle(SAFETY_WRIST_ANGLE);
                robot.claw.open();
                break;

            case WAIT_FOR_INPUT:
                // Waiting for right trigger input
                break;

            case OPEN_CLAW_BEFORE_PICKUP:
                robot.claw.open();
                stepStartTime = System.currentTimeMillis();
                break;

            case WAIT_AFTER_OPENING_CLAW:
                // Waiting for the timer
                break;

            case MOVE_WRIST_TO_PICKUP_POSITION:
                robot.wrist.setAngle(PICKUP_WRIST_ANGLE);
                break;

            case MOVE_ARM_TO_PICKUP_POSITION:
                robot.arm.moveToAngle(PICKUP_ARM_ANGLE);
                break;

            case CLOSE_CLAW:
                robot.claw.close();
                stepStartTime = System.currentTimeMillis();
                break;

            case WAIT_AFTER_CLOSING_CLAW:
                // Waiting for the timer
                break;

            case MOVE_BACK_TO_SAFETY:
                robot.arm.moveToAngle(SAFETY_ARM_ANGLE);
                robot.wrist.setAngle(SAFETY_WRIST_ANGLE);
                break;
        }
    }

    @Override
    public void update() {
        if (!isActive) return;

        switch (currentStep) {
            case MOVE_TO_SAFETY_POSITION:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_INPUT;
                }
                break;

            case WAIT_FOR_INPUT:
                // Waiting for right trigger input
                break;

            case OPEN_CLAW_BEFORE_PICKUP:
                if (robot.claw.isOpen()) {
                    currentStep = Step.WAIT_AFTER_OPENING_CLAW;
                }
                break;

            case WAIT_AFTER_OPENING_CLAW:
                if (System.currentTimeMillis() - stepStartTime >= WAIT_DURATION_MS) {
                    currentStep = Step.MOVE_WRIST_TO_PICKUP_POSITION;
                    executeCurrentStep();
                }
                break;

            case MOVE_WRIST_TO_PICKUP_POSITION:
                if (robot.wrist.isAtTarget()) {
                    currentStep = Step.MOVE_ARM_TO_PICKUP_POSITION;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_TO_PICKUP_POSITION:
                if (robot.arm.isCloseToTarget()) {
                    currentStep = Step.CLOSE_CLAW;
                    executeCurrentStep();
                }
                break;

            case CLOSE_CLAW:
                currentStep = Step.WAIT_AFTER_CLOSING_CLAW;
                break;

            case WAIT_AFTER_CLOSING_CLAW:
                if (System.currentTimeMillis() - stepStartTime >= WAIT_DURATION_MS) {
                    currentStep = Step.MOVE_BACK_TO_SAFETY;
                    executeCurrentStep();
                }
                break;

            case MOVE_BACK_TO_SAFETY:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_INPUT;
                }
                break;
        }
    }

    @Override
    public void onRightTriggerPressed() {
        if (isActive && currentStep == Step.WAIT_FOR_INPUT) {
            currentStep = Step.OPEN_CLAW_BEFORE_PICKUP;
            executeCurrentStep();
        }
    }

    @Override
    public String getCurrentStep() {
        return currentStep.toString();
    }
}