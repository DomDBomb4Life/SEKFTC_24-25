// File: Wrist.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist {

    // Servo
    private Servo wristServo;

    // Servo positions (normalized from 0.0 to 1.0)
    private static final double WRIST_MIN_POSITION = 0.1;     // Adjust based on your hardware
    private static final double WRIST_MAX_POSITION = 0.9;     // Adjust based on your hardware

    private static final double POSITION_THRESHOLD = 0.02; // Threshold for position checking

    // Target angle for the wrist (in degrees)
    private double targetAngle = 0.0;

    // Reference to the Claw subsystem
    private Claw claw;

    // Servo position corresponding to 0 degrees
    private double servoPositionAt0Degrees = 0.0; // Default to midpoint; adjust as needed

    // Constructor
    public Wrist(HardwareMap hardwareMap, Claw claw) {
        // Initialize the servo
        wristServo = hardwareMap.get(Servo.class, "WristServo");

        // Initialize Claw reference
        this.claw = claw;

        // Set initial angle directly without invoking setAngle()
        targetAngle = 0.0;
        updateServoPosition();
    }

    // Set wrist angle (in degrees)
    public void setAngle(double angleDegrees) {
        this.targetAngle = angleDegrees;
        updateServoPosition();

        // Only call setWristAngleOffset if claw is not null
        if (claw != null) {
            claw.setWristAngleOffset(targetAngle);
        }
    }

    // Update servo position based on target angle
    private void updateServoPosition() {
        double position = angleToServoPosition(targetAngle);
        // Clamp position between min and max
        position = Math.max(WRIST_MIN_POSITION, Math.min(position, WRIST_MAX_POSITION));
        wristServo.setPosition(position);
    }

    // Convert angle in degrees to servo position (normalized)
    private double angleToServoPosition(double angleDegrees) {
        // Assuming the servo rotates from 0 to 180 degrees
        double servoRangeDegrees = 180.0; // Adjust if your servo has a different range

        // Calculate position offset from 90 degrees
        double positionOffset = angleDegrees / servoRangeDegrees;

        // Apply the offset to the servo position at 90 degrees
        return servoPositionAt0Degrees + positionOffset;
    }

    // Set the servo position that corresponds to 90 degrees
    public void setServoPositionAt90Degrees(double position) {
        this.servoPositionAt0Degrees = position;
    }

    // Get current angle (in degrees)
    public double getAngle() {
        double position = wristServo.getPosition();
        double servoRangeDegrees = 180.0; // Adjust if your servo has a different range
        double positionOffset = position - servoPositionAt0Degrees;
        return positionOffset * servoRangeDegrees;
    }

    // Get target angle (in degrees)
    public double getTargetAngle() {
        return targetAngle;
    }

    // Manual control method (in degrees)
    public void adjustAngle(double deltaDegrees) {
        setAngle(targetAngle + deltaDegrees);
    }

    // Check if wrist is at target angle
    public boolean isAtTarget() {
        return Math.abs(getAngle() - targetAngle) < POSITION_THRESHOLD * 180.0;
    }

    public double getPosition() {
        return wristServo.getPosition();
    }
}