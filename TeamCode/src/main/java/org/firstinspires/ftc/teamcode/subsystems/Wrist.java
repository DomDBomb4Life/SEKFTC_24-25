// File: Wrist.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist {

    // Servo
    private Servo wristServo;

    // Servo positions (normalized from 0.0 to 1.0)
    private static final double WRIST_MIN_POSITION = 0.0;     // Adjust based on your hardware
    private static final double WRIST_MAX_POSITION = 1.0;     // Adjust based on your hardware

    private static final double POSITION_THRESHOLD = 0.02; // Threshold for position checking

    // Target position
    private double targetPosition = WRIST_MIN_POSITION;

    // Constructor
    public Wrist(HardwareMap hardwareMap) {
        // Initialize the servo
        wristServo = hardwareMap.get(Servo.class, "WristServo");

        // Set initial position
        setPosition(WRIST_MIN_POSITION); // Start at the minimum position
    }

    // Set wrist position (normalized from 0.0 to 1.0)
    public void setPosition(double position) {
        // Clamp the position within limits
        targetPosition = Math.max(WRIST_MIN_POSITION, Math.min(position, WRIST_MAX_POSITION));
        wristServo.setPosition(targetPosition);
    }

    // Manual control method
    public void setManualPosition(double position) {
        setPosition(position);
    }

    // Set wrist angle (in degrees)
    public void setAngle(double angleDegrees) {
        // Convert degrees to servo position
        double position = angleToServoPosition(angleDegrees);
        setPosition(position);
    }

    // Convert angle in degrees to servo position (normalized)
    private double angleToServoPosition(double angleDegrees) {
        // Assuming the servo rotates from 0 to 180 degrees
        double servoRangeDegrees = 180.0; // Adjust if your servo has a different range
        double normalizedPosition = angleDegrees / servoRangeDegrees;

        // Clamp normalized position between 0.0 and 1.0
        return Math.max(WRIST_MIN_POSITION, Math.min(normalizedPosition, WRIST_MAX_POSITION));
    }

    // Get current servo position
    public double getPosition() {
        return wristServo.getPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    // Get current angle (in degrees)
    public double getAngle() {
        // Assuming linear relationship between servo position and angle
        double servoRangeDegrees = 180.0; // Adjust if your servo has a different range
        return wristServo.getPosition() * servoRangeDegrees;
    }

    // Check if wrist is at target position
    public boolean isAtTarget() {
        return Math.abs(wristServo.getPosition() - targetPosition) < POSITION_THRESHOLD;
    }

    // Update method (if needed)
    public void update() {
        // Implement any periodic updates or checks if necessary
    }
}