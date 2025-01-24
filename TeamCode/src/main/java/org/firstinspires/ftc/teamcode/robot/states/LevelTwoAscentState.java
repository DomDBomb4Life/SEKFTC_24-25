package org.firstinspires.ftc.teamcode.robot.states;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class LevelTwoAscentState extends BaseState {
    public enum Step {
        MOVE_ARM_DOWN,
        WAIT_FOR_TRIGGER_1,
        LOCKING_IN,
        WAIT_FOR_TRIGGER_2,
        HANG,
        WAIT_FOR_TRIGGER_3,
        LOWER_VIPERLIFT_TO_LIFT_ROBOT,
        COMPLETED
    }

    private Step currentStep;
    private long waitStartTime;

    // Hypothetical target positions/angles - adjust as needed
    private static final double ARM_HOOK_ANGLE = 38; // Angle to get under hook
    private static final double ARM_UP_ANGLE = 90.0;    // Angle to lift hook
    private static final double WRIST_ANGLE = 90.0;     // Neutral wrist angle
    private static final int VIPERLIFT_HOOK_POS = 3050; // Position to latch
    private static final int VIPERLIFT_LIFT_POS = 2000; // Position to lift robot

    // Constructor
    public LevelTwoAscentState(Robot robot, boolean isAutonomous) {
        super(robot, isAutonomous);
    }

    @Override
    protected void start() {
        // Initial step: move arm down so we can get under the bottom hook
        currentStep = Step.MOVE_ARM_DOWN;
        
        // Viper lift stays at its initial position (assume 0)
        robot.viperLift.moveToPosition(4300);
    }

    @Override
    protected void cleanup() {
        // No special cleanup needed
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {

            case MOVE_ARM_DOWN:
            if (robot.viperLift.isCloseToTarget()) {
                robot.arm.moveToAngle(65);
                robot.wrist.setAngle(10);
                currentStep = Step.WAIT_FOR_TRIGGER_1;
            }
                
                break;

            case COMPLETED:
                // Done
                break;
        }
    }

    @Override
    public void onUserInput(UserInput input) {
        if (!isAutonomous) {
            if (input == UserInput.RIGHT_TRIGGER) {
                // If we're waiting at WAIT_FOR_TRIGGER_1, proceed to LIFT_VIPERLIFT
                if (currentStep == Step.WAIT_FOR_TRIGGER_1 && robot.viperLift.isCloseToTarget()) {
                    robot.viperLift.moveToPosition(1857);
                    robot.arm.oscillate
                    currentStep = Step.LOCKING_IN;
                }
            }
        }
        // In Autonomous mode, no user input is needed
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