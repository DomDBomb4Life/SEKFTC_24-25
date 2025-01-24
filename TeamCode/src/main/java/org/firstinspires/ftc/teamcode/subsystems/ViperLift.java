// File: ViperLift.java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ViperLift {

    // Motors
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;

    // Encoder positions
    private static final int POSITION_MIN = 0;
    private static final int POSITION_MAX = 4500;
    private static final int POSITION_MARGIN = 50;

    // Current target
    private int targetPosition = POSITION_MIN;
    private int encoderOffset = 0;

    // Optional synergy with the Winch
    private Winch attachedWinch = null;
    private boolean synergyEnabled = false;

    public ViperLift(HardwareMap hardwareMap) {
        leftLiftMotor = hardwareMap.get(DcMotor.class, "LiftL");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "LiftR");
        configureMotor(leftLiftMotor);
        configureMotor(rightLiftMotor);

        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();
    }

    private void configureMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders() {
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderOffset = 0;
    }

    public void initializePosition(int position) {
        stop();
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoderOffset = position;

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveToPosition(int position) {
        targetPosition = Math.max(POSITION_MIN, Math.min(position, POSITION_MAX));
        int adjusted = targetPosition - encoderOffset;

        leftLiftMotor.setTargetPosition(adjusted);
        rightLiftMotor.setTargetPosition(adjusted);

        leftLiftMotor.setPower(1.0);
        rightLiftMotor.setPower(1.0);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void adjustTargetPosition(int increment) {
        if (isCloseToTarget()) {
            moveToPosition(targetPosition + increment);
        }
    }

    public void stop() {
        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);
    }

    public int getCurrentPosition() {
        int avg = (leftLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2;
        return avg + encoderOffset;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public boolean isCloseToTarget() {
        return Math.abs(getCurrentPosition() - targetPosition) <= POSITION_MARGIN;
    }

    public void moveToMin() {
        moveToPosition(POSITION_MIN);
    }

    public void moveToMax() {
        moveToPosition(POSITION_MAX);
    }

    // ---------- Winch Synergy -----------
    public void setWinch(Winch w, boolean enable) {
        this.attachedWinch = w;
        this.synergyEnabled = enable;
    }

    // Called each loop
    public void update() {
        // If synergy is enabled, feed the ViperLift position to the Winch's update:
        if (synergyEnabled && attachedWinch != null) {
            attachedWinch.update(getCurrentPosition());
        }
        // Additional ViperLift logic or PID can go here
    }
}