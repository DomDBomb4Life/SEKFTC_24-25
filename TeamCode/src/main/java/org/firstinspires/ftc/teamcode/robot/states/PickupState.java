// File: PickupState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class PickupState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;

    // State active flag
    private boolean isActive = false;

    // Constructor
    public PickupState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
    }

    // Activate the pickup state
    public void activate() {
        isActive = true;
        // Lower Viper Lift if necessary
        viperLift.moveToPosition(0);

        // Move arm and wrist to positions suitable for picking up pieces
        arm.moveToAngle(-14);      // Arm down to pick up
        wrist.setAngle(90);      // Wrist angle for floor level
        claw.open();             // Open claw to grab piece
    }

    // Deactivate the pickup state
    public void deactivate() {
        isActive = false;
    }

    // Update method
    public void update() {
        if (isActive) {
            // Monitor if subsystems have reached target positions
            // Implement any necessary logic, such as closing the claw when ready
            if (arm.isCloseToTarget() && wrist.isAtTarget() && claw.isOpen()) {
                // Optionally add logic to automatically close the claw after conditions are met
            }
        }
    }

    // Method to check if the robot is ready to pick up
    public boolean isReadyToPickUp() {
        return isActive && arm.isCloseToTarget() && wrist.isAtTarget();
    }
}