package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The Claw class represents a robotic claw mechanism.
 */
public class Claw {
    //This is where important variables are
    Servo claw;
    double clawposition;
    OpMode opMode;
    
    /**
     * Constructs a Claw object.
     * @param opMode the OpMode object associated with the claw
     */
    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.get(Servo.class,"GateServo");
        this.opMode = opMode;
        clawposition = 0;
    }
    
    /**
     * Opens or closes the claw based on the gamepad input.
     */
    public void OpenClose(){
        if(opMode.gamepad2.b){
            clawposition = 0.32;
        }else{
            clawposition = 0.38;
        }
        claw.setPosition(clawposition);
    }
}
