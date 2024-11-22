// File: IdleState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class IdleState extends BaseState {

    // Constructor
    public IdleState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        // Position subsystems to idle positions
        robot.viperLift.moveToPosition(0);
        robot.arm.moveToAngle(90);
        robot.wrist.setAngle(90); // Adjust to default idle angle for wrist
        robot.claw.close(); // Adjust to default position for claw
    }

    @Override
    public void update() {
        // No ongoing actions; could add monitoring if needed
    }
}