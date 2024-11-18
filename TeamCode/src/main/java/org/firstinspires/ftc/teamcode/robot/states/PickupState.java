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

    // Steps in the pickup process
    public enum Step {
        MOVE_TO_PICKUP_POSITION,
        WAIT_FOR_DRIVE_FORWARD,
        CLOSE_CLAW,
        LIFT_UP,
        COMPLETED
    }

    private Step currentStep;
    private boolean isAutonomous;
    private boolean isActive;

    // Constructor
    public PickupState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw) {
        this(viperLift, arm, wrist, claw, false);
    }

    public PickupState(ViperLift viperLift, Arm arm, Wrist wrist, Claw claw, boolean isAutonomous) {
        this.viperLift = viperLift;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.isAutonomous = isAutonomous;
        this.currentStep = Step.MOVE_TO_PICKUP_POSITION;
        this.isActive = false;
    }

    // Activate the pickup state
    public void activate() {
        isActive = true;
        currentStep = Step.MOVE_TO_PICKUP_POSITION;
        executeCurrentStep();
    }

    // Deactivate the pickup state
    public void deactivate() {
        isActive = false;
        currentStep = Step.COMPLETED;
    }

    // Execute actions for the current step
    private void executeCurrentStep() {
        switch (currentStep) {
            case MOVE_TO_PICKUP_POSITION:
                // Lower Viper Lift if necessary
                viperLift.moveToPosition(0);

                // Move arm and wrist to positions suitable for picking up pieces
                arm.moveToAngle(-14);      // Arm down to pick up
                wrist.setAngle(90);        // Wrist angle for floor level
                claw.open();               // Open claw to grab piece
                break;

            case WAIT_FOR_DRIVE_FORWARD:
                // Waiting for drive forward to complete (handled in op mode)
                break;

            case CLOSE_CLAW:
                claw.close();
                break;

            case LIFT_UP:
                arm.moveToAngle(90);       // Move arm back to idle position
                viperLift.moveToPosition(0);
                break;

            case COMPLETED:
                // Pickup process completed
                break;

            default:
                break;
        }
    }

    // Update method
    public void update() {
        if (!isActive) return;

        switch (currentStep) {
            case MOVE_TO_PICKUP_POSITION:
                if (arm.isCloseToTarget() && wrist.isAtTarget() && viperLift.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_DRIVE_FORWARD;
                    // In TeleOp, you might wait for user input here
                }
                break;

            case WAIT_FOR_DRIVE_FORWARD:
                // In Autonomous, the op mode will handle driving forward
                // Transition to next step once drive forward is complete
                break;

            case CLOSE_CLAW:
                if (claw.isClosed()) {
                    currentStep = Step.LIFT_UP;
                    executeCurrentStep();
                }
                break;

            case LIFT_UP:
                if (arm.isCloseToTarget()) {
                    currentStep = Step.COMPLETED;
                    deactivate(); // Automatically deactivate after completion
                }
                break;

            case COMPLETED:
                // Do nothing
                break;

            default:
                break;
        }
    }

    // Method to signal that drive forward is complete (called from op mode)
    public void onDriveForwardComplete() {
        if (currentStep == Step.WAIT_FOR_DRIVE_FORWARD) {
            currentStep = Step.CLOSE_CLAW;
            executeCurrentStep();
        }
    }

    // Method to check if the pickup process is completed
    public boolean isCompleted() {
        return currentStep == Step.COMPLETED;
    }
}