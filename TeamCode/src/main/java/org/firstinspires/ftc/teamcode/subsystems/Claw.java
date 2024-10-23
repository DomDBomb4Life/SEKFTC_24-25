// File: Claw.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    // Servo
    private Servo clawServo;

    // Servo positions
    private static final double CLAW_OPEN_POSITION = 0.0;   // Adjust based on your hardware
    private static final double CLAW_CLOSED_POSITION = 1.0; // Adjust based on your hardware

    private static final double POSITION_THRESHOLD = 0.02; // Threshold for position checking

    // Target position
    private double targetPosition = CLAW_CLOSED_POSITION;

    // Constructor
    public Claw(HardwareMap hardwareMap) {
        // Initialize the servo
        clawServo = hardwareMap.get(Servo.class, "ClawServo");

        // Set initial position
        close(); // Start with the claw closed
    }

    // Open the claw
    public void open() {
        setPosition(CLAW_OPEN_POSITION);
    }

    // Close the claw
    public void close() {
        setPosition(CLAW_CLOSED_POSITION);
    }

    // Set claw position
    public void setPosition(double position) {
        // Clamp the position within limits
        targetPosition = Math.max(0.0, Math.min(position, 1.0));
        clawServo.setPosition(targetPosition);
    }

    // Manual control method
    public void setManualPosition(double position) {
        setPosition(position);
    }

    // Get current position
    public double getPosition() {
        return clawServo.getPosition();
    }

    // Check if the claw is open
    public boolean isOpen() {
        return Math.abs(clawServo.getPosition() - CLAW_OPEN_POSITION) < POSITION_THRESHOLD;
    }

    // Check if the claw is closed
    public boolean isClosed() {
        return Math.abs(clawServo.getPosition() - CLAW_CLOSED_POSITION) < POSITION_THRESHOLD;
    }
}