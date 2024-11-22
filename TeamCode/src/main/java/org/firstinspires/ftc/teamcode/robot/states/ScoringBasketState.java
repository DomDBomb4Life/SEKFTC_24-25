// File: ScoringBasketState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class ScoringBasketState extends BaseState {
    // Steps in the scoring basket process
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
    private static final long OPEN_CLAW_DELAY = 200; // Delay in milliseconds

    // Constructor
    public ScoringBasketState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.LIFT_UP;
        executeCurrentStep();
    }

    @Override
    public void deactivate() {
        super.deactivate();
        currentStep = Step.COMPLETED;
    }

    private void executeCurrentStep() {
        switch (currentStep) {
            case LIFT_UP:
                robot.viperLift.moveToPosition(8080);
                break;

            case MOVE_ARM_WRIST:
                robot.arm.moveToAngle(148);
                robot.wrist.setAngle(90);
                break;

            case WAIT_TO_OPEN_CLAW:
                if (isAutonomous) {
                    waitStartTime = System.currentTimeMillis();
                }
                break;

            case OPEN_CLAW:
                robot.claw.open();
                break;

            case WAIT_FOR_CLAW_OPEN:
                waitStartTime = System.currentTimeMillis();
                break;

            case MOVE_ARM_WRIST_BACK:
                robot.arm.moveToAngle(90);
                robot.wrist.setAngle(90);
                break;

            case LOWER_LIFT:
                robot.viperLift.moveToPosition(0);
                break;

            case COMPLETED:
                deactivate();
                break;
        }
    }

    @Override
    public void update() {
        if (!isActive) return;

        switch (currentStep) {
            case LIFT_UP:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_WRIST;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_WRIST:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
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
                if (robot.claw.isOpen()) {
                    currentStep = Step.WAIT_FOR_CLAW_OPEN;
                    executeCurrentStep();
                }
                break;

            case WAIT_FOR_CLAW_OPEN:
                if (System.currentTimeMillis() - waitStartTime >= OPEN_CLAW_DELAY) {
                    currentStep = Step.MOVE_ARM_WRIST_BACK;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_WRIST_BACK:
                if (robot.arm.isCloseToTarget()) {
                    currentStep = Step.LOWER_LIFT;
                    executeCurrentStep();
                }
                break;

            case LOWER_LIFT:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                    executeCurrentStep();
                }
                break;

            case COMPLETED:
                // Process completed
                break;
        }
    }

    @Override
    public void onRightTriggerPressed() {
        if (!isAutonomous && currentStep == Step.WAIT_TO_OPEN_CLAW) {
            currentStep = Step.OPEN_CLAW;
            executeCurrentStep();
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