package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    // Servo
    private Servo clawServo;

    // Servo positions (assuming normalized 0.0 to 1.0)
    private static final double CLAW_MIN_POSITION = 0.0; 
    private static final double CLAW_MAX_POSITION = 1.0;

    // Angles for the claw (in degrees)
    private static final double closed = -10;        // Full close
    private static final double open = 25;           // Open position
    private static final double mediumClosed = 13;  // Added: Medium close position

    private static final double POSITION_THRESHOLD = 0.02;

    private double targetAngle = 0.0;
    private double wristAngleOffset = 0.0;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "ClawServo");
        close(); // Start closed
    }

    public void open() {
        setAngle(open);
    }

    public void close() {
        setAngle(closed);
    }

    // New method for medium close
    public void closeMedium() {
        setAngle(mediumClosed);
    }

    public void setAngle(double angleDegrees) {
        this.targetAngle = angleDegrees;
        updateServoPosition();
    }

    public void setWristAngleOffset(double offsetDegrees) {
        this.wristAngleOffset = offsetDegrees;
        updateServoPosition();
    }

    private void updateServoPosition() {
        double totalAngle = targetAngle + wristAngleOffset;
        double position = angleToServoPosition(totalAngle);
        position = Math.max(CLAW_MIN_POSITION, Math.min(position, CLAW_MAX_POSITION));
        clawServo.setPosition(position);
    }

    private double angleToServoPosition(double angleDegrees) {
        double servoRangeDegrees = 180.0;
        return 1.0 - (angleDegrees / servoRangeDegrees);
    }

    public double getAngle() {
        double position = clawServo.getPosition();
        double servoRangeDegrees = 180.0;
        return (1.0 - position) * servoRangeDegrees - wristAngleOffset;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void adjustAngle(double deltaDegrees) {
        setAngle(targetAngle + deltaDegrees);
    }

    public boolean isOpen() {
        return Math.abs(targetAngle - open) < POSITION_THRESHOLD * 180.0;
    }

    public boolean isClosed() {
        return Math.abs(targetAngle - closed) < POSITION_THRESHOLD * 180.0;
    }

    public double getPosition() {
        return clawServo.getPosition();
    }
}