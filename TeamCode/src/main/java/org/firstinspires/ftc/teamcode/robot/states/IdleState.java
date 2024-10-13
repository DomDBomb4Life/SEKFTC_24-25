// File: IdleState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class IdleState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;

    // Constructor
    public IdleState(ViperLift viperLift, Arm arm) {
        this.viperLift = viperLift;
        this.arm = arm;
    }

    // Activate the idle state
    public void activate() {
        // Viper Lift at position zero
        viperLift.moveToPosition(0);
        // Arm at 90 degrees
        arm.moveToAngle(90);
    }

    // Update method if needed
    public void update() {
        // Monitor if subsystems have reached target positions
        // Implement any necessary logic
    }

    // Method to check if the robot is in idle position
    public boolean isAtIdle() {
        return viperLift.isAtTarget() && arm.isAtTarget();
    }
}