package org.firstinspires.ftc.teamcode.drive;

//the import statements import libraries to access information
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * The DriveTrain class represents the drivetrain of a robot.
 * It controls the movement of the robot using four motors.
 */
public class DriveTrain {
    //this declares motors and variables
    private final DcMotor FrontL, FrontR, BackL, BackR;
    private double speed = .5;
    private double leftStickX, leftStickY, pivot;

    /**
     * Constructs a new DriveTrain object.
     * @param opMode The LinearOpMode object used for hardware mapping.
     */
    public DriveTrain(LinearOpMode opMode){
        //this part tells the code what the motors are
        FrontL = opMode.hardwareMap.get(DcMotor.class, "FrontL");
        BackL = opMode.hardwareMap.get(DcMotor.class, "BackL");
        FrontR = opMode.hardwareMap.get(DcMotor.class, "FrontR");
        BackR = opMode.hardwareMap.get(DcMotor.class, "BackR");
        //this flips certain motors so they all move the right direction
        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Drives the robot based on the input from the gamepad.
     * @param opMode The LinearOpMode object used for gamepad input.
     */
    public void drive(LinearOpMode opMode) {
        //this part updates controller
        leftStickY = opMode.gamepad1.left_stick_y;
        leftStickX = -opMode.gamepad1.left_stick_x;
        pivot = -opMode.gamepad1.right_stick_x;
        updateSpeed(opMode);
        //this parts set power values
        FrontL.setPower((pivot + leftStickX + leftStickY) * speed);
        FrontR.setPower((-pivot + (leftStickY - leftStickX)) * speed);
        BackL.setPower((pivot + (leftStickY - leftStickX)) * speed);
        BackR.setPower((-pivot + leftStickX + leftStickY) * speed);
    }

    //This changes speed when triggers pressed
    private void updateSpeed(LinearOpMode opMode){
        if(opMode.gamepad1.right_trigger == 1){
            speed = 1;
        } else if (opMode.gamepad1.left_trigger == 1) {
            speed = 0.25;
        }else{
            // Default speed
            speed = 0.5;
        }
    }
}

