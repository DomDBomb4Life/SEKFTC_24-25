package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The `PlaneLauncher` class represents a launcher for a plane in a robotics system.
 * It controls the position of the launcher servo to launch or dock the plane.
 */
public class PlaneLauncher {
    Servo launcher;
    boolean planeLaunched = false;
    final double launched = .5;
    final double docked = 0;
    private boolean buttonReleased = true;

    /**
     * Constructs a new `PlaneLauncher` object.
     * @param opMode the OpMode object used to access the hardware map
     */
    public PlaneLauncher(OpMode opMode){
        launcher = opMode.hardwareMap.get(Servo.class,"LauncherServo");
    }

    /**
     * Launches or docks the plane based on the gamepad input.
     * @param opMode the OpMode object used to access the gamepad input
     */
    public void Launch(OpMode opMode){
        if(!opMode.gamepad2.y){
            buttonReleased = true;
        }
        if(opMode.gamepad2.y && buttonReleased) {
            planeLaunched = !planeLaunched;
            buttonReleased = false;
        }

        if(planeLaunched) {
            launcher.setPosition(launched);
        }else{
            launcher.setPosition(docked);
        }
    }
}

