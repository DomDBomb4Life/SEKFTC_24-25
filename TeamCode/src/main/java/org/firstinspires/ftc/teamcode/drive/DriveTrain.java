// File: DriveTrain.java
package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * The DriveTrain class represents the drivetrain of the robot.
 * It controls the movement using four motors in a mecanum wheel configuration.
 */
public class DriveTrain {
    // Motor declarations
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    // Speed multiplier
    private double speedMultiplier = 0.5;

    /**
     * Constructs a new DriveTrain object.
     * @param hardwareMap The HardwareMap object used for hardware mapping.
     */
    public DriveTrain(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "FrontL");
        backLeft = hardwareMap.get(DcMotor.class, "BackL");
        frontRight = hardwareMap.get(DcMotor.class, "FrontR");
        backRight = hardwareMap.get(DcMotor.class, "BackR");

        // Set motor directions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to brake when power is zero
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets the zero power behavior for all drivetrain motors.
     * @param behavior The desired zero power behavior.
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    /**
     * Drives the robot based on the provided joystick inputs.
     * @param leftStickY The forward/backward input.
     * @param leftStickX The left/right strafing input.
     * @param rightStickX The rotation input.
     */
    public void drive(double leftStickY, double leftStickX, double rightStickX) {
        
        // Adjust input values for control remapping flexibility
        double pivot = -rightStickX;
        double adjustedLeftStickX = leftStickX;
        double adjustedLeftStickY = -leftStickY;

        // Calculate motor powers using the adjusted inputs
        double flPower = (adjustedLeftStickY + adjustedLeftStickX + pivot) * speedMultiplier;
        double frPower = (adjustedLeftStickY - adjustedLeftStickX - pivot) * speedMultiplier;
        double blPower = (adjustedLeftStickY - adjustedLeftStickX + pivot) * speedMultiplier;
        double brPower = (adjustedLeftStickY + adjustedLeftStickX - pivot) * speedMultiplier;

        // Normalize motor powers if any exceed the range [-1, 1]
        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                    Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        // Set motor powers
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    /**
     * Drives the robot in field-centric mode.
     * @param xPower       The strafe power.
     * @param yPower       The forward/backward power.
     * @param turnPower    The rotation power.
     * @param robotHeading The current heading of the robot in radians.
     */
    public void driveFieldCentric(double xPower, double yPower, double turnPower, double robotHeading) {
        // Convert field-centric inputs to robot-centric
        double cosA = Math.cos(-robotHeading);
        double sinA = Math.sin(-robotHeading);

        double x = xPower * cosA - yPower * sinA;
        double y = xPower * sinA + yPower * cosA;

        // Mecanum wheel calculations
        double frontLeftPower = x + y + turnPower;
        double frontRightPower = -x + y - turnPower;
        double backLeftPower = -x + y + turnPower;
        double backRightPower = x + y - turnPower;

        // Normalize the wheel speeds
        double max = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        );
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower * speedMultiplier);
        frontRight.setPower(frontRightPower * speedMultiplier);
        backLeft.setPower(backLeftPower * speedMultiplier);
        backRight.setPower(backRightPower * speedMultiplier);
    }

    /**
     * Updates the speed multiplier based on trigger inputs.
     * @param leftTrigger  The input from the left trigger (range 0.0 to 1.0).
     * @param rightTrigger The input from the right trigger (range 0.0 to 1.0).
     */
    public void updateSpeed(double leftTrigger, double rightTrigger) {
        if (rightTrigger > 0.5) {
            speedMultiplier = 1.0; // Fast speed
        } else if (leftTrigger > 0.5) {
            speedMultiplier = 0.25; // Slow speed
        } else {
            speedMultiplier = 0.5; // Default speed
        }
    }

    /**
     * Gets the current speed multiplier.
     * @return The speed multiplier.
     */
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }
}