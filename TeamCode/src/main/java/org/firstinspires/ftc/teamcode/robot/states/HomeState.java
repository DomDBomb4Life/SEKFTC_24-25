// File: HomeState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class HomeState {
    // Subsystems
    private ViperLift viperLift;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;

    // Variations
    public enum Variation {
        VARIATION_1,
        VARIATION_2
    }

    private Variation currentVariation = Variation.VARIATION_1;

    // State active flag
    private boolean isActive = false;

    // Constructor
    public HomeState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
    }

    // Activate the home state
    public void activate() {
        isActive = true;
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

    // Deactivate the home state
    public void deactivate() {
        isActive = false;
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

    // Update method
    public void update() {
        if (isActive) {
            // Monitor if subsystems have reached target positions
            // Implement any necessary logic
        }
    }
}