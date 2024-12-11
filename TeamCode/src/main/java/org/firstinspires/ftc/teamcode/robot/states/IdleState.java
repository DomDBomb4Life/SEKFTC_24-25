package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class IdleState extends BaseState {
    public enum Step {
        IDLE,
        COMPLETED
    }

    private Step currentStep = Step.IDLE;

    public IdleState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        // Reset all subsystems to known idle positions
        robot.viperLift.moveToPosition(0);
        robot.arm.moveToAngle(90);
        robot.wrist.setAngle(90);
        robot.claw.close();
        currentStep = Step.IDLE;
    }

    @Override
    protected void cleanup() {
        // No cleanup needed for idle
    }

    @Override
    public void onUpdate() {
        // Idle does nothing, just waits
    }

    @Override
    public boolean isCompleted() {
        // Idle never truly completes by itself
        return currentStep == Step.COMPLETED;
    }

    @Override
    public String getCurrentStep() {
        return currentStep.toString();
    }
}