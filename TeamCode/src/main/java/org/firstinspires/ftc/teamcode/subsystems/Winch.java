// File: Winch.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The Winch subsystem, responsible for controlling a motor that reels a cable in or out
 * to keep the robot level during advanced ascents. This updated version uses geometry/trigonometry
 * in DETACHED mode to compute a spool position/power as the ViperLift changes.
 */
public class Winch {

    public enum Mode {
        /**
         * The winch hook is still attached to the robot's arm.
         * We keep minimal tension (low power) to avoid rope slack, but not enough to strain.
         */
        ATTACHED,

        /**
         * The hook is detached from the arm and latched onto a rung.
         * We must compute spool extension or power using trig based on the ViperLift’s position.
         */
        DETACHED,

        /**
         * A high-power override mode (final big pull). We reel in the rope at max power.
         */
        OVERRIDE
    }

    private DcMotor winchMotor;
    private Mode currentMode = Mode.ATTACHED;

    // Minimal tension power in ATTACHED mode
    private static final double LOW_TENSION_POWER = 0.05;

    // Maximum power in OVERRIDE mode
    private static final double MAX_POWER = 1.0;

    // If we want to operate in a run-to-position style, we can store target positions
    private int targetPosition = 0;
    private boolean runToPosition = false;

    // -----------------------------
    // Geometry/Trigonometry Fields
    // -----------------------------
    // Hypothetical geometry constants: treat the robot as a rectangle with:
    //   - ViperLift extension on the "left edge."
    //   - The Winch at the "bottom-right corner."
    // We do an example function that calculates spool length from the lift extension.
    private double baseRobotHeight = 300.0;   // in ticks or mm (example)
    private double baseRobotWidth  = 200.0;   // in ticks or mm (example)
    private double spoolTicksPerMM = 2.0;     // how many winch ticks per mm of rope
    /**
     * If the ViperLift extends from 0 to 1000 (arbitrary ticks),
     * we compute diagonal as sqrt( (baseRobotWidth)^2 + (baseRobotHeight + extension)^2 ).
     * This is purely demonstration logic; you will need to tune actual geometry.
     */

    public Winch(HardwareMap hardwareMap) {
        // Initialize the motor
        winchMotor = hardwareMap.get(DcMotor.class, "WinchMotor");
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Main update method: must be called in the robot loop.
     * In DETACHED mode, we do geometry-based spool adjustments.
     * In ATTACHED or OVERRIDE, simpler logic is used.
     *
     * @param viperPosition the current extension of the ViperLift in ticks (0..~11000).
     */
    public void update(int viperPosition) {
        // If we are in runToPosition mode, that typically overrides normal operation:
        if (runToPosition) {
            // If you want to do partial logic or check motor status, do so here
            return;
        }

        switch (currentMode) {
            case ATTACHED:
                // Keep rope taut with minimal power
                winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                winchMotor.setPower(LOW_TENSION_POWER);
                break;

            case DETACHED:
                // Use geometry/trig to find how much spool needed
                winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                int spoolTarget = computeTrigonometricSpoolPosition(viperPosition);
                winchMotor.setTargetPosition(spoolTarget);

                // Use some moderate power for leveling
                // If it's too low, you won't keep level; if too high, it might strain
                winchMotor.setPower(0.6);
                break;

            case OVERRIDE:
                // Final big pull: max power
                winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                winchMotor.setPower(MAX_POWER);
                break;
        }
    }

    /**
     * Example geometry-based formula:
     *   - Let "vertical" = (baseRobotHeight + extension).
     *   - Let "horizontal" = baseRobotWidth.
     *   - Hypotenuse = sqrt(vertical^2 + horizontal^2).
     * Convert that hypotenuse to spool ticks using spoolTicksPerMM.
     */
    private int computeTrigonometricSpoolPosition(int viperLiftTicks) {
        // Convert viperLiftTicks to "extension mm" (placeholder)
        double extensionMM = viperLiftTicks / 10.0; // assume 1 tick = 0.1 mm
        double vertical = baseRobotHeight + extensionMM;
        double horizontal = baseRobotWidth;

        double diagonalMM = Math.sqrt(vertical * vertical + horizontal * horizontal);

        // Convert diagonal mm to spool ticks
        double spoolTicks = diagonalMM * spoolTicksPerMM;

        return (int) Math.round(spoolTicks);
    }

    /**
     * Manual override for user input: spool power.
     * Immediately sets Mode to OVERRIDE so geometry-based logic is suspended.
     */
    public void setPower(double power) {
        currentMode = Mode.OVERRIDE;
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor.setPower(power);
    }

    /**
     * Run to a user-specified spool position (in encoder ticks).
     * This method overrides normal "update" logic until stopRunToPosition() is called.
     */
    public void setTargetPosition(int position) {
        runToPosition = true;
        targetPosition = position;
        winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchMotor.setTargetPosition(position);
        winchMotor.setPower(MAX_POWER);
    }

    /**
     * Cancel runToPosition mode, revert to normal "Mode" logic in update().
     */
    public void stopRunToPosition() {
        runToPosition = false;
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winchMotor.setPower(0.0);
    }

    // ----------- Mode Setters ----------
    public void setModeAttached() {
        runToPosition = false;
        currentMode = Mode.ATTACHED;
    }

    public void setModeDetached() {
        runToPosition = false;
        currentMode = Mode.DETACHED;
    }

    public void setModeOverride() {
        runToPosition = false;
        currentMode = Mode.OVERRIDE;
    }

    // ---------- Accessors -------------
    public Mode getMode() {
        return currentMode;
    }

    public boolean isRunToPosition() {
        return runToPosition;
    }

    public int getCurrentPosition() {
        return winchMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }
}