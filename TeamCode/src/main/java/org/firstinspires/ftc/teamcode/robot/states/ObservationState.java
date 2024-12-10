// File: ObservationState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class ObservationState extends BaseState {
    // Steps in the observation process
    public enum Step {
        LIFT_VIPERLIFT_TO_PRE_HEIGHT,
        MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW,
        WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW,
        CLOSE_THE_CLAW,
        LIFT_VIPERLIFT_TO_SECOND_HEIGHT,
        MOVE_ARM_UP,
        LOWER_VIPERLIFT,
        COMPLETED
    }

    private Step currentStep;
    private long waitStartTime;
    private static final long OPEN_CLAW_DELAY = 250;

    // Constructor
    public ObservationState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.LIFT_VIPERLIFT_TO_PRE_HEIGHT;
        executeCurrentStep();
    }

    @Override
    public void deactivate() {
        super.deactivate();
        currentStep = Step.COMPLETED;
    }

    private void executeCurrentStep() {
        switch (currentStep) {
            case LIFT_VIPERLIFT_TO_PRE_HEIGHT:
                // Raise ViperLift to predetermined height
                robot.viperLift.moveToPosition(553); // Adjust as needed
                break;

            case MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW:
                // Move arm down backwards and open claw
                robot.arm.moveToAngle(167); // Adjust angle as needed for backwards position
                robot.wrist.setAngle(90);   // Adjust wrist angle as needed
                robot.claw.open();
                break;

            case WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW:
                // Waiting for right trigger input
                break;

            case CLOSE_THE_CLAW:
                //i need the claw closed AAAAA
                robot.claw.close();
                waitStartTime = System.currentTimeMillis();
                break;

            case LIFT_VIPERLIFT_TO_SECOND_HEIGHT:
                // Lift ViperLift up more to take specimen off the wall
                robot.viperLift.moveToPosition(553); // Adjust as needed
                break;

            case MOVE_ARM_UP:
                // Move arm back up to idle position
                robot.arm.moveToAngle(90); // Adjust angle as needed
                robot.wrist.setAngle(90);  // Adjust wrist angle as needed
                break;

            case LOWER_VIPERLIFT:
                // Lower ViperLift to the bottom position
                robot.viperLift.moveToPosition(0);
                break;

            case COMPLETED:
                // Process completed; deactivate state
                deactivate();
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    @Override
    public void update() {
        if (!isActive) return;

        switch (currentStep) {
            case LIFT_VIPERLIFT_TO_PRE_HEIGHT:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_DOWN_BACKWARDS_AND_OPEN_CLAW:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW;
                }
                break;

            case WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW:
                // Waiting for right trigger input
                break;

            case CLOSE_THE_CLAW:
                if (System.currentTimeMillis() - waitStartTime >= OPEN_CLAW_DELAY) {
                    currentStep = Step.LIFT_VIPERLIFT_TO_SECOND_HEIGHT;
                    executeCurrentStep();
                }
                break;

            case LIFT_VIPERLIFT_TO_SECOND_HEIGHT:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_UP;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_UP:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.LOWER_VIPERLIFT;
                    executeCurrentStep();
                }
                break;

            case LOWER_VIPERLIFT:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                    executeCurrentStep();
                }
                break;

            case COMPLETED:
                // Do nothing; state remains here
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    @Override
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_RIGHT_TRIGGER_TO_CLOSE_CLAW) {
            currentStep = Step.CLOSE_THE_CLAW;
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