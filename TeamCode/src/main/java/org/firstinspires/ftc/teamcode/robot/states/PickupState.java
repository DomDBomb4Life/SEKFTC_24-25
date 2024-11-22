// File: PickupState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class PickupState extends BaseState {
    // Steps in the pickup process
    public enum Step {
        MOVE_TO_PICKUP_POSITION,
        WAIT_FOR_DRIVE_FORWARD,
        CLOSE_CLAW,
        LIFT_UP,
        COMPLETED
    }

    private Step currentStep;
    private long waitStartTime;
    private static final long OPEN_CLAW_DELAY = 200; // Delay in milliseconds

    // Constructor
    public PickupState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.MOVE_TO_PICKUP_POSITION;
        executeCurrentStep();
    }

    @Override
    public void deactivate() {
        super.deactivate();
        currentStep = Step.COMPLETED;
    }

    private void executeCurrentStep() {
        switch (currentStep) {
            case MOVE_TO_PICKUP_POSITION:
                // Lower Viper Lift if necessary
                robot.viperLift.moveToPosition(0);

                // Move arm and wrist to positions suitable for picking up pieces
                robot.arm.moveToAngle(-20);      // Arm down to pick up
                robot.wrist.setAngle(75);        // Wrist angle for floor level
                robot.claw.open();               // Open claw to grab piece
                break;

            case WAIT_FOR_DRIVE_FORWARD:
                // Waiting for drive forward to complete (handled in op mode)
                break;

            case CLOSE_CLAW:
                robot.claw.close();
                waitStartTime = System.currentTimeMillis();
                break;

            case LIFT_UP:
                robot.arm.moveToAngle(90);       // Move arm back to idle position
                robot.viperLift.moveToPosition(0);
                break;

            case COMPLETED:
                // Pickup process completed
                deactivate();
                break;

            default:
                break;
        }
    }

    @Override
    public void update() {
        if (!isActive) return;

        switch (currentStep) {
            case MOVE_TO_PICKUP_POSITION:
                if (robot.arm.isCloseToTarget() && robot.wrist.isAtTarget() && robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_DRIVE_FORWARD;
                    // In TeleOp, you might wait for user input here
                }
                break;

            case WAIT_FOR_DRIVE_FORWARD:
                // In Autonomous, the op mode will handle driving forward
                // Transition to next step once drive forward is complete
                break;

            case CLOSE_CLAW:
                if (System.currentTimeMillis() - waitStartTime >= OPEN_CLAW_DELAY) {
                    currentStep = Step.LIFT_UP;
                    executeCurrentStep();

                }
                break;

            case LIFT_UP:
                if (robot.arm.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                    deactivate(); // Automatically deactivate after completion
                }
                break;

            case COMPLETED:
                // Do nothing
                break;

            default:
                break;
        }
    }

    // Method to signal that drive forward is complete (called from op mode)
    public void onDriveForwardComplete() {
        if (currentStep == Step.WAIT_FOR_DRIVE_FORWARD) {
            currentStep = Step.CLOSE_CLAW;
            executeCurrentStep();
        }
    }
    @Override
    public void onRightTriggerPressed() {
        if (currentStep == Step.WAIT_FOR_DRIVE_FORWARD) {
            currentStep = Step.CLOSE_CLAW;
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