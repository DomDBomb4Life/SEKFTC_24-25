// File: Winch.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The Winch subsystem, responsible for controlling a motor that reels a cable
 * in or out to maintain appropriate slack or tension.
 */
public class Winch {

    // Winch operating mode
    public enum Mode {
        ATTACHED,   // minimal tension
        DETACHED,   // automatically adjusts spool for Slack
        OVERRIDE    // user manually controls spool power
    }

    private DcMotor winchMotor;
    private Mode currentMode = Mode.DETACHED;

    // Minimal tension power in ATTACHED mode
    private static final double LOW_TENSION_POWER = 0.05;

    // Maximum power in OVERRIDE mode
    private static final double MAX_POWER = 1.0;

    // If we want to operate in a run-to-position style, we can store target positions
    private int targetPosition = 0;
    public boolean runToPosition = true;

    // -----------------------------
    // Slack-based constants
    // -----------------------------
    // We want:   ViperLift=0     => Winch=704
    //            ViperLift=3993  => Winch=8528
    private static final int VIPER_MIN = 0;
    private static final int VIPER_MAX = 3993;
    private static final int WINCH_MIN = 704;
    private static final int WINCH_MAX = 8528;

    /**
     * Constructor
     */
    public Winch(HardwareMap hardwareMap) {
        // Initialize the motor
        winchMotor = hardwareMap.get(DcMotor.class, "WinchMotor");
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Main update method: must be called repeatedly (e.g. in TeleOp loop).
     * Depending on the Mode, we either set minimal tension, keep Slack automatically,
     * or let the user override with manual power.
     *
     * @param viperPosition the current extension of the ViperLift in ticks (0..~3993).
     */
    public void update(int viperPosition) {
        // If we are in runToPosition mode, that overrides normal operation:

        if(runToPosition == true) {
            targetPosition = computeSlackSpoolPosition(viperPosition);
            winchMotor.setTargetPosition(targetPosition);
            winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Use a moderate power to move spool
            winchMotor.setPower(1.0);
        }else{
            winchMotor.setTargetPosition(targetPosition);
            winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Use a moderate power to move spool
            winchMotor.setPower(1.0);
        }
    }
    public void adjustTargetAngle(int increment) {
        targetPosition += increment;
    }

    /**
     * Given the ViperLift ticks, linearly map to the needed spool position
     * to maintain slack:
     *
     *   ViperLift=0    => Winch=704
     *   ViperLift=3993 => Winch=8528
     *
     * We'll clamp the lift range just in case.
     */
    private int computeSlackSpoolPosition(int viperLiftTicks) {
        // 1) Clamp
        int clamped = Math.max(VIPER_MIN, Math.min(VIPER_MAX, viperLiftTicks));

        // 2) Linear interpolation from [VIPER_MIN..VIPER_MAX] => [WINCH_MIN..WINCH_MAX]
        double slope = (double)(WINCH_MAX - WINCH_MIN) / (double)(VIPER_MAX - VIPER_MIN);
        double spool = WINCH_MIN + slope * (clamped - VIPER_MIN);

        // Round to nearest integer
        return (int)Math.round(spool);
    }

    public void manualPower(double power){
        if(runToPosition == false) {
            winchMotor.setPower(power);
        }

    }

    public void manual(){
        winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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