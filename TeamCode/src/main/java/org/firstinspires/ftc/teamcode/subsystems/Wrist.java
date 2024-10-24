// File: Wrist.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist {

    // Servo
    private Servo wristServo;

    // Servo positions
    private static final double POSITION_MIN = 0.0;
    private static final double POSITION_MAX = 1.0;

    // Target position
    private double targetPosition = POSITION_MIN;

    // Constructor
    public Wrist(HardwareMap hardwareMap) {
        // Initialize the servo
        wristServo = hardwareMap.get(Servo.class, "WristServo");
        setPosition(POSITION_MIN);
    }

    // Set wrist position
    public void setPosition(double position) {
        targetPosition = Math.max(POSITION_MIN, Math.min(position, POSITION_MAX));
        wristServo.setPosition(targetPosition);
    }

    // Manual control
    public void setManualPosition(double position) {
        setPosition(position);
    }

    // Set wrist angle (assuming 0 to 180 degrees)
    public void setAngle(double angleDegrees) {
        double position = angleDegrees / 180.0;
        setPosition(position);
    }

    // Get current position
    public double getPosition() {
        return wristServo.getPosition();
    }

    // Check if at target
    public boolean isAtTarget() {
        return Math.abs(wristServo.getPosition() - targetPosition) < 0.01;
    }

    // Update method if needed
    public void update() {
        // Implement any necessary logic
    }
}