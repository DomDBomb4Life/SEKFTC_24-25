// File: Arm.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Arm {

    // Motor
    private DcMotorEx armMotor;

    // Angle limits (in degrees)
    private static final double ANGLE_MIN = -20.0;
    private static final double ANGLE_MAX = 180.0;
    private static final double ANGLE_MARGIN = 2.0; // Margin for angle tolerance in degrees

    // Encoder counts per revolution (CPR) for the motor
    private static final double ENCODER_CPR = 1993.6; // Adjust based on your motor (e.g., Neverest 40)
    private static final double GEAR_RATIO = 1.0; // Adjust if gears are used
    private static final double COUNTS_PER_DEGREE = (ENCODER_CPR * GEAR_RATIO) / 360.0;

    // Zero position angle (where encoder count is zero)
    private double ZERO_POSITION_ANGLE = -12.2; //TODO

    // Target angle
    private double targetAngle = ZERO_POSITION_ANGLE; // Start at -18 degrees as default

    // Constructor
    public Arm(HardwareMap hardwareMap, Boolean isAutonomous) {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotorEx.class, "Arm");


        if (isAutonomous){
            ZERO_POSITION_ANGLE = 157;
        }

        // Motor configuration
        configureMotor(armMotor);
    }
    public void OverridePosition(){
        ZERO_POSITION_ANGLE = 136.3;
    }

    private void configureMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set PIDF coefficients for RUN_TO_POSITION
        PIDFCoefficients pidfCoefficients = motor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Adjust the PIDF coefficients as needed
        pidfCoefficients.p = 10.0; // Increase P for tighter control
        pidfCoefficients.i = 0.0;
        pidfCoefficients.d = 0.0;
        pidfCoefficients.f = 0.0;

        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);

        // Set mode to RUN_USING_ENCODER
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Initialize the arm encoder
    public void initializeEncoder(double zeroPos) {
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ZERO_POSITION_ANGLE = zeroPos;

    }

    // Move to a specific angle
    public void moveToAngle(double angle) {
        // Clamp angle within bounds
        targetAngle = Math.max(ANGLE_MIN, Math.min(angle, ANGLE_MAX));

        // Calculate target encoder counts based on the calibrated zero position
        int targetPosition = angleToCounts(targetAngle);

        // Set target position
        armMotor.setTargetPosition(targetPosition);

        // Apply power and set mode
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5); // Use a moderate power for smooth movement
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

    // Adjust target angle incrementally
    public void adjustTargetAngle(double increment) {
        moveToAngleStrong(targetAngle + increment);
    }

    // Convert angle to encoder counts
    private int angleToCounts(double angle) {
        return (int) ((angle - ZERO_POSITION_ANGLE) * COUNTS_PER_DEGREE);
    }

    // Convert encoder counts to angle
    private double countsToAngle(int counts) {
        return (counts / COUNTS_PER_DEGREE) + ZERO_POSITION_ANGLE;
    }

    // Stop the arm
    public void stop() {
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
        if(isCloseToTarget()) {
            armMotor.setPower(1.0);
        }
    }

    // Set power to the arm motor (e.g., to make it go limp)
    public void setPower(double power) {
        armMotor.setPower(power);
        if (power == 0.0) {
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        } else {
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }
}