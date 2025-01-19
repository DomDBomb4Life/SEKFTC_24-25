// File: LevelThreeAscentState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * A state for performing a Level 3 Ascent which uses both the ViperLift and Winch
 * to keep the robot level at all times. The steps are hypothetical placeholders and can be
 * adjusted for your actual mechanical design and timing.
 */
public class LevelThreeAscentState extends BaseState {
    public enum Step {
        ATTACH_WINCH,           // Keep tension, ensure it's attached
        WAIT_FOR_TRIGGER_1,     // Wait for user input before detaching
        DETACH_WINCH,           // Detach from the arm so the hook is on the bar
        SYNC_WITH_VIPER,        // Winch + ViperLift working together to keep the robot level
        WAIT_FOR_TRIGGER_2,     // Wait for user input to override
        OVERRIDE_PULL,          // The final big pull up
        COMPLETED
    }

    private Step currentStep;

    public LevelThreeAscentState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        // Step 1: ensure the winch is in ATTACHED mode, minimal tension
        currentStep = Step.ATTACH_WINCH;
        robot.winch.setModeAttached(); 
    }

    @Override
    protected void cleanup() {
        // No special cleanup if we remain on the bar
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {
            case ATTACH_WINCH:
                // Possibly ensure the rope is taut. We do that automatically in the Winch's update.
                // Move to WAIT_FOR_TRIGGER_1 (TeleOp prompt) or skip in Autonomous.
                currentStep = isAutonomous ? Step.DETACH_WINCH : Step.WAIT_FOR_TRIGGER_1;
                break;

            case WAIT_FOR_TRIGGER_1:
                // Wait for operator to confirm we detach the winch from the arm
                break;

            case DETACH_WINCH:
                // We forcibly set the mode to DETACHED, so the hook stays on the top rung.
                // Now the rope is controlling the angle in conjunction with the ViperLift.
                robot.winch.setModeDetached();
                currentStep = Step.SYNC_WITH_VIPER;
                break;

            case SYNC_WITH_VIPER:
                // Winch + ViperLift are automatically coordinated via update logic. 
                // We'll wait for user trigger if in TeleOp, or skip if in autonomous.
                currentStep = isAutonomous ? Step.OVERRIDE_PULL : Step.WAIT_FOR_TRIGGER_2;
                break;

            case WAIT_FOR_TRIGGER_2:
                // Wait for user input to do final big pull
                break;

            case OVERRIDE_PULL:
                // Final big pull. Winch is at full power to raise the robot.
                robot.winch.setModeOverride();
                // Possibly also do a ViperLift move if needed.
                // Once done, we set to COMPLETED or rely on user input again.
                currentStep = Step.COMPLETED;
                break;

            case COMPLETED:
                // Done. We remain in override or any final state needed.
                break;
        }
    }

    @Override
    public void onUserInput(UserInput input) {
        if (!isAutonomous) {
            if (input == UserInput.RIGHT_TRIGGER) {
                switch (currentStep) {
                    case WAIT_FOR_TRIGGER_1:
                        // Switch to DETACH_WINCH
                        currentStep = Step.DETACH_WINCH;
                        break;
                    case WAIT_FOR_TRIGGER_2:
                        // Switch to OVERRIDE_PULL
                        currentStep = Step.OVERRIDE_PULL;
                        break;
                    default:
                        break;
                }
            }
        }
    }

    @Override
    public boolean isCompleted() {
        return currentStep == Step.COMPLETED;
    }

    @Override
    public String getCurrentStep() {
        return currentStep.toString();
    }
}