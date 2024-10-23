// File: DriveTrain.java
package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The DriveTrain class represents the drivetrain of a robot.
 * It controls the movement of the robot using four motors.
 */
public class DriveTrain {
    // Motors
    private final DcMotor FrontL, FrontR, BackL, BackR;
    private double speed = 0.5;

    /**
     * Constructs a new DriveTrain object.
     * @param hardwareMap The hardware map for initializing motors.
     */
    public DriveTrain(HardwareMap hardwareMap){
        // Initialize motors
        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        BackR = hardwareMap.get(DcMotor.class, "BackR");
        // Reverse direction for left motors
        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Drives the robot based on the input from the gamepad.
     * @param leftStickY Left stick Y input.
     * @param leftStickX Left stick X input.
     * @param rightStickX Right stick X input (pivot).
     * @param leftTrigger Left trigger input.
     * @param rightTrigger Right trigger input.
     */
    public void drive(double leftStickY, double leftStickX, double rightStickX, double leftTrigger, double rightTrigger) {
        updateSpeed(leftTrigger, rightTrigger);

        double pivot = -rightStickX;
        double adjustedLeftStickX = -leftStickX;
        double adjustedLeftStickY = leftStickY;

        // Set power values
        double frontLPower = (pivot + adjustedLeftStickX + adjustedLeftStickY) * speed;
        double frontRPower = (-pivot + (adjustedLeftStickY - adjustedLeftStickX)) * speed;
        double backLPower = (pivot + (adjustedLeftStickY - adjustedLeftStickX)) * speed;
        double backRPower = (-pivot + adjustedLeftStickX + adjustedLeftStickY) * speed;

        FrontL.setPower(frontLPower);
        FrontR.setPower(frontRPower);
        BackL.setPower(backLPower);
        BackR.setPower(backRPower);
    }

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
    
        FrontL.setPower(frontLeftPower * speed);
        FrontR.setPower(frontRightPower * speed);
        BackL.setPower(backLeftPower * speed);
        BackR.setPower(backRightPower * speed);
    }

    //This changes speed when triggers pressed
    private void updateSpeed(double leftTrigger, double rightTrigger){
        if(rightTrigger > 0.5){
            speed = 1.0;
        } else if (leftTrigger > 0.5) {
            speed = 0.25;
        } else {
            // Default speed
            speed = 0.5;
        }
    }

    // Get current speed
    public double getSpeed() {
        return speed;
    }

    // Methods to get motor powers for telemetry
    public double getFrontLPower() {
        return FrontL.getPower();
    }

    public double getFrontRPower() {
        return FrontR.getPower();
    }

    public double getBackLPower() {
        return BackL.getPower();
    }

    public double getBackRPower() {
        return BackR.getPower();
    }
}