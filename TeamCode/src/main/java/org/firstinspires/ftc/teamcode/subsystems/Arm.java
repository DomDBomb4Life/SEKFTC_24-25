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
    private static final double ANGLE_MARGIN = 3.0; // Margin for angle tolerance in degrees

    // Encoder counts per revolution (CPR) for the motor
    private static final double ENCODER_CPR = 1425.1; // Adjust based on your motor (e.g., Neverest 40)
    private static final double GEAR_RATIO = 1.0; // Adjust if gears are used
    private static final double COUNTS_PER_DEGREE = (ENCODER_CPR * GEAR_RATIO) / 360.0;

    // Target angle
    private double targetAngle = 90.0; // Start at 90 degrees as default

    // Constructor
    public Arm(HardwareMap hardwareMap) {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotor.class, "Arm");

        // Motor configuration
        configureMotor(armMotor);

        // Set initial position to interpret the current position as 90 degrees
        calibrateStartingPosition();
    }

    private void configureMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Calibrate the encoder to treat the current position as 90 degrees
    private void calibrateStartingPosition() {
        // Set the encoder’s current position as equivalent to 90 degrees
        int initialCounts = angleToCounts(90.0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(initialCounts);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move to a specific angle
    public void moveToAngle(double angle) {
        // Clamp angle within bounds
        targetAngle = Math.max(ANGLE_MIN, Math.min(angle, ANGLE_MAX));

        // Calculate target encoder counts based on the calibrated zero position
        int targetPosition = angleToCounts(targetAngle);

        // Set target position
        armMotor.setTargetPosition(targetPosition);

        // Apply power
        armMotor.setPower(0.5); // Use a moderate power for smooth movement

        // Run to position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Adjust target angle incrementally
    public void adjustTargetAngle(double increment) {
        if (isCloseToTarget()) {
            moveToAngle(targetAngle + increment);
        }
    }

    // Convert angle to encoder counts
    private int angleToCounts(double angle) {
        return (int) ((angle - 90) * COUNTS_PER_DEGREE);
    }

    // Convert encoder counts to angle
    private double countsToAngle(int counts) {
        return (counts / COUNTS_PER_DEGREE) + 90;
    }

    // Stop the arm
    public void stop() {
        armMotor.setPower(0);
    }

    // Get current angle
    public double getCurrentAngle() {
        int currentPosition = armMotor.getCurrentPosition();
        return countsToAngle(currentPosition);
    }

    // Get target angle
    public double getTargetAngle() {
        return targetAngle;
    }

    // Check if arm is close to the target angle
    public boolean isCloseToTarget() {
        double currentAngle = getCurrentAngle();
        return Math.abs(currentAngle - targetAngle) <= ANGLE_MARGIN;
    }

    // Update method to be called periodically (if needed)
    public void update() {
        // Implement PID control or additional logic if necessary
    }
}