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
    private static final int ENCODER_CPR = 1120; // Adjust based on your motor (e.g., Neverest 40)
    private static final double GEAR_RATIO = 1.0; // Adjust if gears are used
    private static final double COUNTS_PER_DEGREE = (ENCODER_CPR * GEAR_RATIO) / 360.0;

    // Target angle
    private double targetAngle = 0.0;

    // Constructor
    public Arm(HardwareMap hardwareMap) {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotor.class, "Arm");

        // Motor configuration
        configureMotor(armMotor);

        // Set initial position
        resetEncoder();
    }

    private void configureMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Reset encoder
    public void resetEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move to a specific angle
    public void moveToAngle(double angle) {
        // Clamp angle within bounds
        targetAngle = Math.max(ANGLE_MIN, Math.min(angle, ANGLE_MAX));

        // Calculate target encoder counts
        int targetPosition = angleToCounts(targetAngle);

        // Set target position
        armMotor.setTargetPosition(targetPosition);

        // Set mode to run to position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power (adjust as necessary)
        armMotor.setPower(0.5); // Use a moderate power for smooth movement
    }

    // Convert angle to encoder counts
    private int angleToCounts(double angle) {
        return (int) (angle * COUNTS_PER_DEGREE);
    }

    // Convert encoder counts to angle
    private double countsToAngle(int counts) {
        return counts / COUNTS_PER_DEGREE;
    }

    // Stop the arm
    public void stop() {
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Get current angle
    public double getCurrentAngle() {
        int currentPosition = armMotor.getCurrentPosition();
        return countsToAngle(currentPosition);
    }

    // Check if arm is at target angle
    public boolean isAtTarget() {
        return !armMotor.isBusy();
    }

    // Preset positions for different states
    public void moveToHomePosition() {
        moveToAngle(0.0); // Home position angle
    }

    public void moveToScoringBasketPosition() {
        moveToAngle(180.0); // Scoring basket position angle
    }

    public void moveToHangingPosition() {
        moveToAngle(90.0); // Hanging position angle
    }

    // Update method to be called periodically (if needed)
    public void update() {
        // Implement PID control or additional logic if necessary
    }
}