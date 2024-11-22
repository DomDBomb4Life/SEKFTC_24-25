// File: ScoringSpecimenState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class ScoringSpecimenState extends BaseState {
    // Steps in the scoring specimen process
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
    private static final long OPEN_CLAW_DELAY = 250;

    // Constructor
    public ScoringSpecimenState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.LIFT_VIPERLIFT_UP;
        executeCurrentStep();
    }

    @Override
    public void deactivate() {
        super.deactivate();
        currentStep = Step.COMPLETED;
    }

    private void executeCurrentStep() {
        switch (currentStep) {
            case LIFT_VIPERLIFT_UP:
                // Lift the ViperLift up to a high position
                robot.viperLift.moveToPosition(3500); // Adjust this value as needed
                break;

            case MOVE_ARM_DOWN_FORWARDS:
                // Move the arm down forwards to hang the specimen
                robot.arm.moveToAngle(38);   // Adjust angle as needed
                robot.wrist.setAngle(110);   // Adjust angle as needed
                // Assuming claw is already holding the specimen
                break;

            case WAIT_FOR_RIGHT_TRIGGER:
                // Waiting for right trigger input
                break;

            case MOVE_VIPER_LIFT_UP_TO_HOOK:
                // Rotate arm back a predetermined amount to hook the specimen
                robot.viperLift.moveToPosition(3750); // Adjust angle as needed
                // Claw remains closed
                break;

            case OPEN_CLAW:
                // Open the claw to release the specimen
                robot.claw.open();
                waitStartTime = System.currentTimeMillis();
                break;

            case MOVE_ARM_UP:
                // Move arm back up to idle position
                robot.arm.moveToAngle(90);   // Adjust angle as needed
                robot.wrist.setAngle(90);    // Adjust angle as needed
                break;

            case LOWER_VIPERLIFT_TO_BOTTOM:
                // Lower the ViperLift to the bottom position
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
            case LIFT_VIPERLIFT_UP:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_DOWN_FORWARDS;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_DOWN_FORWARDS:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.WAIT_FOR_RIGHT_TRIGGER;
                    // Do not executeCurrentStep() because we're waiting for input
                }
                break;

            case WAIT_FOR_RIGHT_TRIGGER:
                // Waiting for right trigger input
                break;

            case MOVE_VIPER_LIFT_UP_TO_HOOK:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.OPEN_CLAW;
                    executeCurrentStep();
                }
                break;

            case OPEN_CLAW:
                if (System.currentTimeMillis() - waitStartTime >= OPEN_CLAW_DELAY) {
                    currentStep = Step.MOVE_ARM_UP;
                    executeCurrentStep();
                }
                break;

            case MOVE_ARM_UP:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget()) {
                    currentStep = Step.LOWER_VIPERLIFT_TO_BOTTOM;
                    executeCurrentStep();
                }
                break;

            case LOWER_VIPERLIFT_TO_BOTTOM:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                    executeCurrentStep();
                }
                break;

            case COMPLETED:
                // Process completed; state remains here
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    @Override
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_RIGHT_TRIGGER) {
            currentStep = Step.MOVE_VIPER_LIFT_UP_TO_HOOK;
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