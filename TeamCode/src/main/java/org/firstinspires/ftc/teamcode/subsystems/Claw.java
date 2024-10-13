// File: Claw.java
// Package declaration
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    // Servo
    private Servo clawServo;

    // Servo positions
    private static final double CLAW_OPEN_POSITION = 0.0;   // Adjust based on your hardware
    private static final double CLAW_CLOSED_POSITION = 1.0; // Adjust based on your hardware

    // Constructor
    public Claw(HardwareMap hardwareMap) {
        // Initialize the servo
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Set initial position
        close(); // Start with the claw closed
    }

    // Open the claw
    public void open() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }

    // Close the claw
    public void close() {
        clawServo.setPosition(CLAW_CLOSED_POSITION);
    }

    // Get current position
    public double getPosition() {
        return clawServo.getPosition();
    }

    // Check if the claw is open
    public boolean isOpen() {
        return clawServo.getPosition() == CLAW_OPEN_POSITION;
    }

    // Check if the claw is closed
    public boolean isClosed() {
        return clawServo.getPosition() == CLAW_CLOSED_POSITION;
    }

    // Update method (if needed for future enhancements)
    public void update() {
        // Implement any periodic updates if necessary
    }
}