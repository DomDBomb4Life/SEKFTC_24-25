// File: IdleState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * Represents the idle state of the robot.
 */
public class IdleState {

    // Subsystems
    private final ViperLift viperLift;
    private final Arm arm;

    // Constructor
    public IdleState(ViperLift viperLift, Arm arm) {
        this.viperLift = viperLift;
        this.arm = arm;
    }

    // Activate the idle state
    public void activate() {
        // Viper Lift at position zero
        viperLift.moveToPosition(0);
        // Arm at 90 degrees (neutral position)
        arm.moveToAngle(90);
    }

    // Update method if needed
    public void update() {
        // Could add logic to maintain idle state if necessary
    }

    // Method to check if the robot is in idle position
    public boolean isAtIdle() {
        return viperLift.isAtTarget() && arm.isAtTarget();
    }
}