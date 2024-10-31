// File: ViperLift.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ViperLift {

    // Motors
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;

    // Encoder positions
    private static final int POSITION_MIN = 0;         // Fully retracted position
    private static final int POSITION_MAX = 11700;     // Fully extended position (example value)
    private static final int POSITION_MARGIN = 50;     // Margin for position tolerance

    // Target position
    private int targetPosition = POSITION_MIN;

    // Constructor
    public ViperLift(HardwareMap hardwareMap) {
        // Initialize motors
        leftLiftMotor = hardwareMap.get(DcMotor.class, "LiftL");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "LiftR");

        // Motor configurations
        configureMotor(leftLiftMotor);
        configureMotor(rightLiftMotor);

        // Change the direction of one of the motors if necessary
        leftLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set initial positions
        resetEncoders();
    }

    private void configureMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Reset encoders
    public void resetEncoders() {
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move to a specific position
    public void moveToPosition(int position) {
        // Clamp position within bounds
        targetPosition = Math.max(POSITION_MIN, Math.min(position, POSITION_MAX));

        // Set target position for both motors
        leftLiftMotor.setTargetPosition(targetPosition);
        rightLiftMotor.setTargetPosition(targetPosition);

        // Apply power
        leftLiftMotor.setPower(1.0);
        rightLiftMotor.setPower(1.0);

        // Set motors to run to position
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Adjust target position incrementally
    public void adjustTargetPosition(int increment) {
        if (isCloseToTarget()) {
            moveToPosition(targetPosition + increment);
        }
    }

    // Stop the lift
    public void stop() {
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
    }

    // Get current position
    public int getCurrentPosition() {
        // Average the positions of both motors
        return (leftLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2;
    }

    // Get target position
    public int getTargetPosition() {
        return targetPosition;
    }

    // Check if lift is close to the target position
    public boolean isCloseToTarget() {
        int currentPosition = getCurrentPosition();
        return Math.abs(currentPosition - targetPosition) <= POSITION_MARGIN;
    }

    // Update method to be called in the main loop (if needed)
    public void update() {
        // Implement PID control or other control logic here if necessary
    }

    // Additional methods for preset positions
    public void moveToMin() {
        moveToPosition(POSITION_MIN);
    }

    public void moveToMax() {
        moveToPosition(POSITION_MAX);
    }
}