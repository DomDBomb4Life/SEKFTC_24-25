// File: Arm.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    // Motor
    private DcMotor armMotor;

    // Angle limits (in degrees)
    private static final double ANGLE_MIN = 0.0;
    private static final double ANGLE_MAX = 180.0;

    // Encoder counts per revolution (CPR) for the motor
    private static final int ENCODER_CPR = 538; // Your motor's CPR value

    // Target encoder position
    private int targetPosition = 0;

    // Constructor
    public Arm(HardwareMap hardwareMap) {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotor.class, "Arm");

        // Motor configuration
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();
    }

    // Reset encoder
    public void resetEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Move to a specific angle (in degrees)
    public void moveToAngle(double angle) {
        // Clamp angle within bounds
        double clampedAngle = Math.max(ANGLE_MIN, Math.min(angle, ANGLE_MAX));

        // Convert angle to encoder counts (assuming 1 revolution = 360 degrees)
        targetPosition = (int) ((clampedAngle / 360.0) * ENCODER_CPR);

        // Set target position and power
        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(0.5);
    }

    // Adjust target angle incrementally
    public void adjustTargetAngle(double increment) {
        moveToAngle(getCurrentAngle() + increment);
    }

    // Get current angle (in degrees) from encoder counts
    public double getCurrentAngle() {
        return (armMotor.getCurrentPosition() * 360.0) / ENCODER_CPR;
    }

    // Check if arm is at target position
    public boolean isAtTarget() {
        return !armMotor.isBusy();
    }

    // Update method if needed
    public void update() {
        // Implement any necessary logic
    }
}