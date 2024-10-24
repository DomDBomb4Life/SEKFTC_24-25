// File: HomeState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

/**
 * Represents the home position state of the robot.
 */
public class HomeState {

    // Subsystems
    private final ViperLift viperLift;
    private final Arm arm;
    private final Wrist wrist;

    // Variations for different home positions
    public enum Variation {
        DEFAULT,
        ALTERNATE
    }

    private Variation currentVariation = Variation.DEFAULT;

    // Constructor
    public HomeState(ViperLift viperLift, Arm arm, Wrist wrist) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
    }

    // Activate the home state
    public void activate() {
        // Viper Lift down
        viperLift.moveToPosition(0);

        // Set positions based on current variation
        switch (currentVariation) {
            case DEFAULT:
                arm.moveToAngle(0);
                wrist.setAngle(90);
                break;
            case ALTERNATE:
                arm.moveToAngle(15);
                wrist.setAngle(0);
                break;
        }
    }

    // Switch between variations
    public void switchVariation() {
        if (currentVariation == Variation.DEFAULT) {
            currentVariation = Variation.ALTERNATE;
        } else {
            currentVariation = Variation.DEFAULT;
        }

        // Re-activate to apply the new variation
        activate();
    }

    // Get current variation
    public Variation getCurrentVariation() {
        return currentVariation;
    }

    // Update method if needed
    public void update() {
        // Monitor if subsystems have reached target positions
        // Could add logic here if necessary
    }
}