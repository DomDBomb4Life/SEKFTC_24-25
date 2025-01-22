// File: LevelThreeAscentState.java
package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Refined Level 3 Ascent with more detailed steps:
 *   1. Attach winch to high rung
 *   2. Attach arm hook to low rung
 *   3. Attach side panel hook to low rung
 *   4. Detach arm hook from low rung
 *   5. Attach arm hook to top rung
 *   6. Retract everything for final ascend
 */
public class LevelThreeAscentState extends BaseState {

    public enum Step {
        // Step 1: Attach Winch to High Rung
        LIFT_VIPERLIFT_FULL,         // Lift the ViperLift up
        MOVE_ARM_DOWN,               // Move arm down
        WAIT_FOR_TRIGGER_1,          // Wait for user input
        MOVE_VIPERLIFT_DOWN,         // Move ViperLift partially down
        MOVE_ARM_BACK,               // Move arm back
        MOVE_VIPERLIFT_MIDDLE,       // Move to a mid position, end of Step 1

        // Step 2: Attach Arm Hook to Low Rung
        MOVE_VIPERLIFT_LOW,          // Move ViperLift down nearly to bottom
        MOVE_ARM_FORWARD,            // Attach hook forward
        WAIT_FOR_TRIGGER_2,          // Wait for user input
        RETRACT_VIPERLIFT_WINCH_SYNC,// Retract both to lock in low rung
        WAIT_FOR_TRIGGER_3,          // Wait for user

        // Step 3: Attach Side Panel Hook to Low Rung (Simplified)
        ENGAGE_SIDE_PANEL_HOOK,      // Possibly any needed movements
        WAIT_FOR_TRIGGER_4,          // Wait for user to confirm it's latched

        // Step 4: Detach Arm Hook from Low Rung
        DETACH_ARM_HOOK,             // Move arm away
        WAIT_FOR_TRIGGER_5,

        // Step 5: Attach Arm Hook to Top Rung
        MOVE_VIPERLIFT_UP_A_BIT,
        MOVE_ARM_90,                 // Move arm to 90 deg
        WAIT_FOR_TRIGGER_6,

        // Step 6: Complete final ascend
        RETRACT_ALL_FOR_FINAL_ASCENT,
        COMPLETED
    }

    private Step currentStep;

    public LevelThreeAscentState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        currentStep = Step.LIFT_VIPERLIFT_FULL;
        // Hypothetical positions
        robot.viperLift.moveToPosition(3000);   // Lift up to 
        // Winch attach mode to rung or minimal tension 
        robot.winch.setModeAttached();
    }

    @Override
    protected void cleanup() {
        // Possibly finalize
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {
            case LIFT_VIPERLIFT_FULL:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_DOWN;
                    robot.arm.moveToAngle(20);
                }
                break;

            case MOVE_ARM_DOWN:
                if (robot.arm.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_TRIGGER_1;
                }
                break;

            case WAIT_FOR_TRIGGER_1:
                // Wait user input
                break;

            case MOVE_VIPERLIFT_DOWN:
                robot.viperLift.moveToPosition(5000);
                currentStep = Step.MOVE_ARM_BACK;
                break;

            case MOVE_ARM_BACK:
                if (robot.viperLift.isCloseToTarget()) {
                    robot.arm.moveToAngle(120);
                    currentStep = Step.MOVE_VIPERLIFT_MIDDLE;
                }
                break;

            case MOVE_VIPERLIFT_MIDDLE:
                // Move to mid
                robot.viperLift.moveToPosition(4000);
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_VIPERLIFT_LOW;
                    robot.viperLift.moveToPosition(500); 
                }
                break;

            case MOVE_VIPERLIFT_LOW:
                if (robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.MOVE_ARM_FORWARD;
                    robot.arm.moveToAngle(-10);
                }
                break;

            case MOVE_ARM_FORWARD:
                if (robot.arm.isCloseToTarget()) {
                    currentStep = Step.WAIT_FOR_TRIGGER_2;
                }
                break;

            case WAIT_FOR_TRIGGER_2:
                // Wait user input
                break;

            case RETRACT_VIPERLIFT_WINCH_SYNC:
                // Enable synergy or DETACHED if we want geometry-based approach
                robot.winch.setModeDetached(); 
                // Move ViperLift to 0 while the winch re-adjusts automatically
                robot.viperLift.moveToPosition(0);
                currentStep = Step.WAIT_FOR_TRIGGER_3;
                break;

            case WAIT_FOR_TRIGGER_3:
                // Wait user input
                break;

            case ENGAGE_SIDE_PANEL_HOOK:
                // Possibly do minimal movements or a wait
                currentStep = Step.WAIT_FOR_TRIGGER_4;
                break;

            case WAIT_FOR_TRIGGER_4:
                // Wait user input
                break;

            case DETACH_ARM_HOOK:
                // Move arm away from rung
                robot.arm.moveToAngle(90);
                currentStep = Step.WAIT_FOR_TRIGGER_5;
                break;

            case WAIT_FOR_TRIGGER_5:
                // Wait user input
                break;

            case MOVE_VIPERLIFT_UP_A_BIT:
                // Move ViperLift up partially
                robot.viperLift.moveToPosition(2000);
                currentStep = Step.MOVE_ARM_90;
                break;

            case MOVE_ARM_90:
                if (robot.viperLift.isCloseToTarget()) {
                    robot.arm.moveToAngle(90);
                    currentStep = Step.WAIT_FOR_TRIGGER_6;
                }
                break;

            case WAIT_FOR_TRIGGER_6:
                // Wait user input
                break;

            case RETRACT_ALL_FOR_FINAL_ASCENT:
                // final step: override pull if needed
                robot.winch.setModeOverride();
                robot.viperLift.moveToPosition(0);
                currentStep = Step.COMPLETED;
                break;

            case COMPLETED:
                // done
                break;
        }
    }

    @Override
    public void onUserInput(UserInput input) {
        if (!isAutonomous && input == UserInput.RIGHT_TRIGGER) {
            switch (currentStep) {
                case WAIT_FOR_TRIGGER_1:
                    currentStep = Step.MOVE_VIPERLIFT_DOWN;
                    break;

                case WAIT_FOR_TRIGGER_2:
                    currentStep = Step.RETRACT_VIPERLIFT_WINCH_SYNC;
                    break;

                case WAIT_FOR_TRIGGER_3:
                    currentStep = Step.ENGAGE_SIDE_PANEL_HOOK;
                    break;

                case WAIT_FOR_TRIGGER_4:
                    currentStep = Step.DETACH_ARM_HOOK;
                    break;

                case WAIT_FOR_TRIGGER_5:
                    currentStep = Step.MOVE_VIPERLIFT_UP_A_BIT;
                    break;

                case WAIT_FOR_TRIGGER_6:
                    currentStep = Step.RETRACT_ALL_FOR_FINAL_ASCENT;
                    break;

                default:
                    break;
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