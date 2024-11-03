// File: ObservationState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ObservationState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;

    // State active flag
    private boolean isActive = false;

    // Constructor
    public ObservationState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
    }

    // Activate the observation state
    public void activate() {
        isActive = true;
        // Slightly raise Viper Lift
        viperLift.moveToPosition(500); // Adjust as needed
        // Set arm and wrist positions
        arm.moveToAngle(180);
        wrist.setAngle(90); // Adjust as needed
    }

    // Deactivate the observation state
    public void deactivate() {
        isActive = false;
    }

    // Update method
    public void update() {
        if (isActive) {
            // Allow manual control of the claw in TeleOpMode
            // Additional logic can be added here if necessary
        }
    }

    // Check if the state is active
    public boolean isActive() {
        return isActive;
    }
}