// File: HomeState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class HomeState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;
    private Wrist wrist;

    // Variations
    public enum Variation {
        VARIATION_1,
        VARIATION_2
    }

    private Variation currentVariation = Variation.VARIATION_1;

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
            case VARIATION_1:
                arm.moveToAngle(0);
                wrist.setAngle(90);
                break;
            case VARIATION_2:
                arm.moveToAngle(15);
                wrist.setAngle(0);
                break;
        }
    }

    // Switch between variations
    public void switchVariation() {
        if (currentVariation == Variation.VARIATION_1) {
            currentVariation = Variation.VARIATION_2;
        } else {
            currentVariation = Variation.VARIATION_1;
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
        // Implement any necessary logic
    }
}

