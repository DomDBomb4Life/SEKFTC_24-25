// File: ViperLift.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ViperLift {

    // Motors
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;

    // Encoder positions
<<<<<<< HEAD
    private static final int POSITION_MIN = 0;        // Fully retracted position
    private static final int POSITION_MAX = 11600;    // Fully extended position
=======
    private static final int POSITION_MIN = 0;         // Fully retracted position
    private static final int POSITION_MAX = 11700;     // Fully extended position (example value)
>>>>>>> Test

    // Target position
    private int targetPosition = POSITION_MIN;

    // Constructor
    public ViperLift(HardwareMap hardwareMap) {
        // Initialize motors
        leftLiftMotor = hardwareMap.get(DcMotor.class, "LiftL");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "LiftR");

        // Motor configurations
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

<<<<<<< HEAD
        // Reverse one motor if necessary
=======
        // Change the direction of one of the motors if necessary
>>>>>>> Test
        leftLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();
    }

<<<<<<< HEAD
=======
    private void configureMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

>>>>>>> Test
    // Reset encoders
    public void resetEncoders() {
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Move to a specific position
    public void moveToPosition(int position) {
        // Clamp position within bounds
        targetPosition = Math.max(POSITION_MIN, Math.min(position, POSITION_MAX));

        // Set target position and power
        leftLiftMotor.setTargetPosition(targetPosition);
        rightLiftMotor.setTargetPosition(targetPosition);
<<<<<<< HEAD
=======

        // Apply power
>>>>>>> Test
        leftLiftMotor.setPower(1.0);
        rightLiftMotor.setPower(1.0);
    }

    // Adjust target position incrementally
    public void adjustTargetPosition(int increment) {
<<<<<<< HEAD
        moveToPosition(targetPosition + increment);
=======
        if (!isBusy()) {
            moveToPosition(targetPosition + increment);
        }
    }

    // Stop the lift
    public void stop() {
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
>>>>>>> Test
    }

    // Get current position
    public int getCurrentPosition() {
        return (leftLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2;
    }

    // Get target position
    public int getTargetPosition() {
        return targetPosition;
    }

    // Check if lift is at target position
    public boolean isAtTarget() {
        return !leftLiftMotor.isBusy() && !rightLiftMotor.isBusy();
    }

<<<<<<< HEAD
    // Update method if needed
=======
    // Check if lift is busy moving
    public boolean isBusy() {
        return leftLiftMotor.isBusy() || rightLiftMotor.isBusy();
    }

    // Update method to be called in the main loop (if needed)
>>>>>>> Test
    public void update() {
        // Implement any necessary logic
    }

    // Move to min and max positions
    public void moveToMin() {
        moveToPosition(POSITION_MIN);
    }

    public void moveToMax() {
        moveToPosition(POSITION_MAX);
    }
}