package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class BaseState implements State {
    protected Robot robot;
    protected boolean isAutonomous;
    protected boolean isActive = false;

    public BaseState(Robot robot, boolean isAutonomous) {
        this.robot = robot;
        this.isAutonomous = isAutonomous;
    }

    @Override
    public final void onEnter() {
        isActive = true;
        start();
    }

    @Override
    public final void onExit() {
        isActive = false;
        cleanup();
    }

    // start() for initializing state logic on entry
    protected abstract void start();

    // cleanup() for resetting any resources on exit
    protected abstract void cleanup();

    @Override
    public void onUserInput(UserInput input) {
        // Default empty, states override if needed
    }

    @Override
    public boolean isCompleted() {
        // Default false, states override once their step = COMPLETED
        return false;
    }

    @Override
    public String getCurrentStep() {
        return "N/A";
    }
}