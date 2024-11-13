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

    // Steps for the two-step movement
    private enum Step {
        MOVE_TO_SAFE_POSITION,
        MOVE_TO_FINAL_POSITION,
        COMPLETED
    }

    private Step currentStep = Step.MOVE_TO_SAFE_POSITION;

    // Safe positions to prevent hitting the floor
    private static final double SAFE_ARM_ANGLE = 0.0; // Adjust as needed
    private static final double SAFE_WRIST_ANGLE = 120.0; // Adjust as needed

    // Final positions based on variation
    private double targetArmAngle;
    private double targetWristAngle;

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
        currentStep = Step.MOVE_TO_SAFE_POSITION;

        // Viper Lift down
        viperLift.moveToPosition(0);

        // Set final positions based on current variation
        switch (currentVariation) {
            case VARIATION_1:
                targetArmAngle = -17;
                targetWristAngle = 96;
                break;
            case VARIATION_2:
                targetArmAngle = -5;
                targetWristAngle = 156;
                break;
        }

        // Move to safe positions first
        arm.moveToAngle(SAFE_ARM_ANGLE);
        wrist.setAngle(SAFE_WRIST_ANGLE);
    }

    // Deactivate the home state
    public void deactivate() {
        isActive = false;
        currentStep = Step.COMPLETED;
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
            switch (currentStep) {
                case MOVE_TO_SAFE_POSITION:
                    // Check if arm and wrist have reached safe positions
                    if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                        // Proceed to final positions
                        arm.moveToAngle(targetArmAngle);
                        wrist.setAngle(targetWristAngle);
                        currentStep = Step.MOVE_TO_FINAL_POSITION;
                    }
                    break;

                case MOVE_TO_FINAL_POSITION:
                    // Check if arm and wrist have reached final positions
                    if (arm.isCloseToTarget() && wrist.isAtTarget()) {
                        currentStep = Step.COMPLETED;
                    }
                    break;

                case COMPLETED:
                    // Do nothing; home state is complete
                    break;
            }
        }
    }

    // Check if home state is completed
    public boolean isCompleted() {
        return currentStep == Step.COMPLETED;
    }
}