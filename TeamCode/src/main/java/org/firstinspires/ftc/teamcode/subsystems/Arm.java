// File: Arm.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Arm subsystem for controlling the robot's arm motor.
 */
public class Arm {

    // Motor
    private DcMotorEx armMotor;

    // Angle limits (in degrees)
    private static final double ANGLE_MIN = -20.0;
    private static final double ANGLE_MAX = 180.0;
    private static final double ANGLE_MARGIN = 2.0; // Margin for angle tolerance in degrees

    // Encoder counts per revolution (CPR) for the motor
    private static final double ENCODER_CPR = 1993.6; // Adjust based on your motor
    private static final double GEAR_RATIO = 1.0;     // Adjust if gears are used
    private static final double COUNTS_PER_DEGREE = (ENCODER_CPR * GEAR_RATIO) / 360.0;

    // Zero position angle (where encoder count is zero)
    private double zeroPositionAngle = -12.2; // TODO: Calibrate this value

    // Target angle
    private double targetAngle = zeroPositionAngle; // Start at zero position by default

    // Oscillation parameters
    private boolean isOscillating = false;
    private double oscillateAngle1 = 0.0;
    private double oscillateAngle2 = 0.0;
    private boolean moveToFirstAngle = true;

    // Constructor
    public Arm(HardwareMap hardwareMap, boolean isAutonomous) {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotorEx.class, "Arm");
        if (isAutonomous) {
            zeroPositionAngle = 157.0;
        }
        // Motor configuration
        configureMotor();
    }

    // Configure the motor settings
    private void configureMotor() {
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients for RUN_TO_POSITION
        PIDFCoefficients pidfCoefficients = armMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Adjust the PIDF coefficients as needed
        pidfCoefficients.p = 10.0; // Increase P for tighter control
        pidfCoefficients.i = 0.0;
        pidfCoefficients.d = 0.0;
        pidfCoefficients.f = 0.0;
        armMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
    }

    // Initialize the arm encoder
    public void initializeEncoder(double zeroPos) {
        zeroPositionAngle = zeroPos;
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void OverridePosition(){
        ZERO_POSITION_ANGLE = 136.3;
    }

    public void moveToAngleStrong(double angle) {
        // Clamp angle within bounds
        targetAngle = Math.max(ANGLE_MIN, Math.min(angle, ANGLE_MAX));

        // Calculate target encoder counts based on the calibrated zero position
        int targetPosition = angleToCounts(targetAngle);

        // Set target position
        armMotor.setTargetPosition(targetPosition);

        // Apply power and set mode
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0); // Use a moderate power for smooth movement
    }

    // Move to a specific angle
    public void moveToAngle(double angle) {
        // Cancel oscillation if active
        if (isOscillating) {
            isOscillating = false;
        }
        // Clamp angle within bounds
        targetAngle = clampAngle(angle);
        // Calculate target encoder counts based on the calibrated zero position
        int targetPosition = angleToCounts(targetAngle);
        // Set target position
        armMotor.setTargetPosition(targetPosition);
        // Apply power and set mode
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0); // Use full power for movement
    }

    // Adjust target angle incrementally
    public void adjustTargetAngle(double increment) {
        moveToAngle(targetAngle + increment);
    }

    // Oscillate between two angles
    public void oscillate(double angle1, double angle2) {
        // Set oscillation parameters
        isOscillating = true;
        oscillateAngle1 = clampAngle(angle1);
        oscillateAngle2 = clampAngle(angle2);
        moveToFirstAngle = true;
        // Start oscillation by moving to the first angle
        moveToAngle(oscillateAngle1);
    }

    // Clamp angle within the minimum and maximum limits
    private double clampAngle(double angle) {
        return Math.max(ANGLE_MIN, Math.min(angle, ANGLE_MAX));
    }

    // Convert angle to encoder counts
    private int angleToCounts(double angle) {
        return (int) ((angle - zeroPositionAngle) * COUNTS_PER_DEGREE);
    }

    // Convert encoder counts to angle
    private double countsToAngle(int counts) {
        return (counts / COUNTS_PER_DEGREE) + zeroPositionAngle;
    }

    // Stop the arm
    public void stop() {
        isOscillating = false; // Cancel oscillation if active
        armMotor.setPower(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

    // Update method to be called periodically
    public void update() {
        if (isOscillating) {
            if (isCloseToTarget()) {
                // Switch to the other angle
                if (moveToFirstAngle) {
                    moveToAngle(oscillateAngle2);
                } else {
                    moveToAngle(oscillateAngle1);
                }
                // Toggle the flag
                moveToFirstAngle = !moveToFirstAngle;
            }
        }
    }

    // Set power to the arm motor (e.g., to make it go limp)
    public void setPower(double power) {
        isOscillating = false; // Cancel oscillation if active
        armMotor.setPower(power);
        if (power == 0.0) {
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        } else {
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }
}