package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing independently of the corresponding
 * slot's motor direction.
 */
public class Encoder {
    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private final DcMotorEx motor;
    private Direction direction;

    public Encoder(DcMotorEx motor) {
        this.motor = motor;
        this.direction = Direction.FORWARD;
    }

    public Encoder(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap.get(DcMotorEx.class, deviceName));
    }

    public Direction getDirection() {
        return direction;
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state.
     *
     * @param direction either reverse or forward depending on if encoder counts should be negated.
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    private int getMultiplier() {
        int multiplier = direction.getMultiplier();
        if (motor.getDirection() == DcMotorSimple.Direction.REVERSE) {
            multiplier *= -1;
        }
        return multiplier;
    }

    /**
     * Gets the current position of the encoder adjusted for the set direction.
     *
     * @return encoder position.
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition() * getMultiplier();
    }

    /**
     * Gets the corrected velocity of the encoder adjusted for the set direction.
     *
     * @return corrected velocity.
     */
    public double getCorrectedVelocity() {
        return motor.getVelocity() * getMultiplier();
    }
}