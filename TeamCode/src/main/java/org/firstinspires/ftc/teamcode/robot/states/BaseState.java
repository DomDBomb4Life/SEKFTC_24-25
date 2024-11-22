// File: BaseState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class BaseState {
    protected Robot robot;
    protected boolean isActive = false;
    protected boolean isAutonomous = false;

    // Constructor
    public BaseState(Robot robot, boolean isAutonomous) {
        this.robot = robot;
        this.isAutonomous = isAutonomous;
    }

    // Activate the state
    public void activate() {
        isActive = true;
        start();
    }

    // Deactivate the state
    public void deactivate() {
        isActive = false;
    }

    // Abstract methods to be implemented by subclasses
    protected abstract void start();

    public abstract void update();

    // Optional method to handle right trigger input
    public void onRightTriggerPressed() {}

    // Optional method to handle primary button input
    public void onPrimaryButtonPressed() {}

    // Method to check if the state is completed
    public boolean isCompleted() {
        return false;
    }

    // For telemetry or debugging
    public String getCurrentStep() {
        return "N/A";
    }
}