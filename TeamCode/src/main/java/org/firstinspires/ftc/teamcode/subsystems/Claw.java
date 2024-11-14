// File: Claw.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    // Servo
    private Servo clawServo;

    // Servo positions (normalized from 0.0 to 1.0)
    private static final double CLAW_MIN_POSITION = 0.0;     // Adjust based on your hardware
    private static final double CLAW_MAX_POSITION = 1.0;     // Adjust based on your hardware

    private static final double closed = -40.0;

    private static final double open = 4.0;



    private static final double POSITION_THRESHOLD = 0.02; // Threshold for position checking

    // Target angle for the claw (in degrees)
    private double targetAngle = 0.0; // Closed position

    // Wrist angle offset
    private double wristAngleOffset = 0.0;

    // Constructor
    public Claw(HardwareMap hardwareMap) {
        // Initialize the servo
        clawServo = hardwareMap.get(Servo.class, "ClawServo");

        // Set initial position
        close(); // Start with the claw closed
    }

    // Open the claw
    public void open() {
        setAngle(open); // Adjust angle as needed for open position
    }

    // Close the claw
    public void close() {
        setAngle(closed); // Closed position
    }

    // Set claw angle (in degrees)
    public void setAngle(double angleDegrees) {
        this.targetAngle = angleDegrees;
        updateServoPosition();
    }

    // Set wrist angle offset (called from Wrist class)
    public void setWristAngleOffset(double offsetDegrees) {
        this.wristAngleOffset = offsetDegrees;
        updateServoPosition();
    }

    // Update servo position based on target angle and wrist offset
    private void updateServoPosition() {
        double totalAngle = targetAngle + wristAngleOffset;
        double position = angleToServoPosition(totalAngle);
        // Clamp position between min and max
        position = Math.max(CLAW_MIN_POSITION, Math.min(position, CLAW_MAX_POSITION));
        clawServo.setPosition(position);
    }

    // Convert angle in degrees to servo position (normalized)
    private double angleToServoPosition(double angleDegrees) {
        // Assuming the servo rotates from 0 to 180 degrees
        double servoRangeDegrees = 180.0; // Adjust if your servo has a different range
        // Flip direction by subtracting from 1.0
        return 1.0 - (angleDegrees / servoRangeDegrees);
    }

    // Get current angle (in degrees)
    public double getAngle() {
        double position = clawServo.getPosition();
        double servoRangeDegrees = 180.0; // Adjust if your servo has a different range
        return (1.0 - position) * servoRangeDegrees - wristAngleOffset;
    }

    // Get target angle (in degrees)
    public double getTargetAngle() {
        return targetAngle;
    }

    // Manual control method (in degrees)
    public void adjustAngle(double deltaDegrees) {
        setAngle(targetAngle + deltaDegrees);
    }

    // Check if the claw is open
    public boolean isOpen() {
        return Math.abs(targetAngle - open) < POSITION_THRESHOLD * 180.0;
    }

    // Check if the claw is closed
    public boolean isClosed() {
        return Math.abs(targetAngle - closed) < POSITION_THRESHOLD * 180.0;
    }
    public double getPosition() {
        return clawServo.getPosition();
    }
}