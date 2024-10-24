// File: HangingState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

/**
 * Represents the state of the robot when performing the hanging sequence.
 */
public class HangingState {

    // Subsystems
    private final ViperLift viperLift;
    private final Arm arm;

    // Steps in the hanging process
    public enum Step {
        INITIAL_POSITION,
        ARM_EXTEND,
        LIFT_RAISE,
        ARM_ADJUST,
        COMPLETED
    }

    private Step currentStep;

    // Constructor
    public HangingState(ViperLift viperLift, Arm arm) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.currentStep = Step.INITIAL_POSITION;
    }

    // Start the hanging process
    public void start() {
        currentStep = Step.INITIAL_POSITION;
        executeCurrentStep();
    }

    // Progress to the next step based on button presses
    public void progress() {
        switch (currentStep) {
            case INITIAL_POSITION:
                currentStep = Step.ARM_EXTEND;
                break;
            case ARM_EXTEND:
                currentStep = Step.LIFT_RAISE;
                break;
            case LIFT_RAISE:
                currentStep = Step.ARM_ADJUST;
                break;
            case ARM_ADJUST:
                currentStep = Step.COMPLETED;
                break;
            default:
                // Already completed or invalid step
                break;
        }
        executeCurrentStep();
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case INITIAL_POSITION:
                // Move Viper Lift up slightly
                viperLift.moveToPosition(500); // Adjust the position value as needed
                break;

            case ARM_EXTEND:
                // Extend arm to 0 degrees
                arm.moveToAngle(0);
                break;

            case LIFT_RAISE:
                // Raise Viper Lift to maximum height
                viperLift.moveToMax();
                break;

            case ARM_ADJUST:
                // Adjust arm to approximately 80 degrees
                arm.moveToAngle(80);
                break;

            case COMPLETED:
                // Hanging process completed
                // Optionally, you can reset to initial position or stay in completed state
                break;

            default:
                // Handle unexpected cases
                break;
        }
    }

    // Update method to be called periodically
    public void update() {
        // Check if the action for the current step is completed
        boolean actionCompleted = false;

        switch (currentStep) {
            case INITIAL_POSITION:
                actionCompleted = viperLift.isAtTarget();
                break;

            case ARM_EXTEND:
                actionCompleted = arm.isAtTarget();
                break;

            case LIFT_RAISE:
                actionCompleted = viperLift.isAtTarget();
                break;

            case ARM_ADJUST:
                actionCompleted = arm.isAtTarget();
                break;

            case COMPLETED:
                actionCompleted = true;
                break;

            default:
                break;
        }

        // Automatically progress to the next step if the current action is completed
        if (actionCompleted && currentStep != Step.COMPLETED) {
            progress();
        }
    }

    // Get the current step
    public Step getCurrentStep() {
        return currentStep;
    }
}