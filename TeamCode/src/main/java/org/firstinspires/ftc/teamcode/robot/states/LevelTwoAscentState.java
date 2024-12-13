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
        currentStep = Step.WAIT_FOR_TRIGGER_1;
        robot.arm.moveToAngle(14);
        robot.wrist.setAngle(90.0);
        // Viper lift stays at its initial position (assume 0)
        robot.viperLift.moveToPosition(1912);
    }

    @Override
    protected void cleanup() {
        // No special cleanup needed
    }

    @Override
    public void onUpdate() {
        if (!isActive) return;

        switch (currentStep) {

            case WAIT_FOR_TRIGGER_1:
                // In TeleOp, wait for user input (RIGHT_TRIGGER)
                // In Autonomous, we never hit this step because we skip to LIFT_VIPERLIFT directly
                break;

            case LOCKING_IN:
                // Move ViperLift up to hook
                if(robot.viperLift.isCloseToTarget()){
                    robot.arm.moveToAngle(38);
                }
                currentStep =Step.MOVE_ARM_DOWN;
                break;

            case MOVE_ARM_DOWN:
                if (robot.arm.isCloseToTarget()) {
                    robot.arm.moveToAngleStrong(55);
                    robot.viperLift.moveToPosition(2317);
                    currentStep = Step.WAIT_FOR_TRIGGER_2;
                }
                break;

            case WAIT_FOR_TRIGGER_2:
                // Wait for user input in TeleOp to proceed
                break;

            case HANG:
                // Raise the arm so backside hooks grip onto bar
                if (robot.viperLift.isCloseToTarget()) {
                    robot.arm.moveToAngleStrong(80);
                    robot.viperLift.moveToPosition(230);
                    currentStep = Step.WAIT_FOR_TRIGGER_3;
                }
                break;


            case LOWER_VIPERLIFT_TO_LIFT_ROBOT:
                // Lower ViperLift slightly to lift robot off ground
                // Once close to target, mark completed
                if (robot.viperLift.isCloseToTarget()) {;
                }
                break;

            case WAIT_FOR_TRIGGER_3:
                // Wait for user input in TeleOp to proceed
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
                    robot.viperLift.moveToPosition(3050);
                    currentStep = Step.LOCKING_IN;
                }
                // If we're waiting at WAIT_FOR_TRIGGER_2, proceed to RAISE_ARM_FOR_HOOK
                else if (currentStep == Step.WAIT_FOR_TRIGGER_2 && robot.viperLift.isCloseToTarget()) {
                    robot.arm.moveToAngleStrong(74);
                    robot.viperLift.moveToPosition(1858);
                    currentStep = Step.HANG;

                }
                // If we're waiting at WAIT_FOR_TRIGGER_3, proceed to LOWER_VIPERLIFT_TO_LIFT_ROBOT
                else if (currentStep == Step.WAIT_FOR_TRIGGER_3 && robot.viperLift.isCloseToTarget()) {
                    currentStep = Step.LOWER_VIPERLIFT_TO_LIFT_ROBOT;
                    robot.viperLift.moveToPosition(1635);
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