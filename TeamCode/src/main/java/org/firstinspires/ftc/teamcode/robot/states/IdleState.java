// File: IdleState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class IdleState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;

    // State active flag
    private boolean isActive = false;

    // Constructor
    public IdleState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
    }

    // Activate the idle state
    public void activate() {
        isActive = true;
        // Position subsystems to idle positions
        viperLift.moveToPosition(0);
        arm.moveToAngle(90);
        wrist.setAngle(90); // Adjust to default idle angle for wrist
        claw.close(); // Adjust to default position for claw
    }

    // Deactivate the idle state
    public void deactivate() {
        isActive = false;
    }

    // Update method for monitoring subsystem positions if needed
    public void update() {
        if (isActive) {
            // Could add additional monitoring logic here if necessary
        }
    }

    // Method to check if the robot is fully at the idle position
    public boolean isAtIdle() {
        return viperLift.isCloseToTarget() && arm.isCloseToTarget() && wrist.isAtTarget() && claw.isClosed();
    }
}