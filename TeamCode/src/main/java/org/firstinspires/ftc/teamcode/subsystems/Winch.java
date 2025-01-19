// File: Winch.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The Winch subsystem, responsible for controlling a motor that reels a cable in or out
 * to help keep the robot level during advanced ascents (like Level 3). This winch can have
 * different modes (attached vs. detached) and needs to track the ViperLift position.
 */
public class Winch {

    public enum Mode {
        /**
         * The winch hook is still attached to the robot's arm (or some other part).
         * This means the winch simply keeps tension with minimal power to hold the rope taut,
         * preventing slack but not aggressively pulling.
         */
        ATTACHED,

        /**
         * The hook is detached from the arm and latched onto a rung, so the rope is now
         * controlling the robot's orientation in conjunction with the ViperLift. We must keep
         * the robot level by adjusting the winch power or position as the ViperLift changes.
         */
        DETACHED,

        /**
         * A high-power override mode for the final pull. In this mode, the winch applies
         * maximum power to reel the rope in quickly for the final ascend/lift.
         */
        OVERRIDE
    }

    private DcMotor winchMotor;
    private Mode currentMode = Mode.ATTACHED;

    // The low tension power to keep the rope taut in ATTACHED mode
    private static final double LOW_TENSION_POWER = 0.05;

    // The maximum power for detaching or final override
    private static final double MAX_POWER = 1.0;

    // If we need a mapping from ViperLift position to Winch motor position,
    // we can store a ratio or offset. Adjust these as needed for your geometry.
    private double viperToWinchRatio = 0.1;  // Hypothetical ratio for DETACHED mode

    // Target positions for the motor if we want a RUN_TO_POSITION style (optional)
    private int targetPosition = 0;
    private boolean runToPosition = false;

    public Winch(HardwareMap hardwareMap) {
        // Initialize the motor
        winchMotor = hardwareMap.get(DcMotor.class, "WinchMotor");
        // Basic config
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Main update method to be called each loop cycle.
     * If we are in runToPosition mode, or if we want to coordinate with ViperLift,
     * we do it here. Otherwise, we rely on direct power calls.
     *
     * @param viperPosition the current position of the ViperLift, used to keep level
     */
    public void update(int viperPosition) {
        switch (currentMode) {
            case ATTACHED:
                // Keep the rope tight with minimal power. 
                // If needed, we could set a small negative/positive offset to reel in or out a bit.
                winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                winchMotor.setPower(LOW_TENSION_POWER);
                break;

            case DETACHED:
                // Attempt to keep the robot level. 
                // For demonstration, we simply set target power as a function of ViperLift position.
                // Example: if ViperLift goes up, we spool the winch in proportion to viperPosition.
                // This is just a placeholder formula; you'd tune it for your geometry.
                winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                double desiredPower = computeLevelPower(viperPosition);
                winchMotor.setPower(desiredPower);
                break;

            case OVERRIDE:
                // Reels in the rope at maximum power
                winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                winchMotor.setPower(MAX_POWER);
                break;
        }
    }

    /**
     * Helper method to compute how much power we want for DETACHED mode
     * based on the ViperLift position. We might do a simple ratio or a PID approach.
     */
    private double computeLevelPower(int viperPosition) {
        // Example: just do a linear function scaled by viperToWinchRatio
        // to spool in or let out in proportion. 
        // This might be negative if viperPosition is small, or positive if it's large, etc.
        // We'll clamp it to [-1, 1] for safety.
        double rawPower = viperPosition * viperToWinchRatio / 1000.0; // just a random scale
        if (rawPower > 1.0) rawPower = 1.0;
        if (rawPower < -1.0) rawPower = -1.0;
        return rawPower;
    }

    /**
     * Command the winch to run at user-specified power. (Manual override)
     */
    public void setPower(double power) {
        currentMode = Mode.OVERRIDE;
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor.setPower(power);
    }

    /**
     * If using RUN_TO_POSITION style, set a new position to spool to (optional).
     */
    public void setTargetPosition(int position) {
        runToPosition = true;
        targetPosition = position;
        winchMotor.setTargetPosition(position);
        winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchMotor.setPower(MAX_POWER);
    }

    /**
     * End runToPosition mode and revert to normal control.
     */
    public void stopRunToPosition() {
        runToPosition = false;
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor.setPower(0.0);
    }

    /**
     * Switch to ATTACHED mode.
     */
    public void setModeAttached() {
        currentMode = Mode.ATTACHED;
    }

    /**
     * Switch to DETACHED mode.
     */
    public void setModeDetached() {
        currentMode = Mode.DETACHED;
    }

    /**
     * Switch to OVERRIDE mode (final big pull).
     */
    public void setModeOverride() {
        currentMode = Mode.OVERRIDE;
    }

    public Mode getMode() {
        return currentMode;
    }
}