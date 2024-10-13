package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.*;

/**
 * This class represents the DriveV2 OpMode, which is used for controlling the robot during teleop mode.
 */
@TeleOp(name = "Drive V2")
public class DriveV2 extends LinearOpMode {
    // Initiating the classes
    private Claw claw;
    private ArmPipeline arm;
    private PlaneLauncher launcher;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // Create the robot classes
        DriveTrain mecanumWheels = new DriveTrain(this);


        waitForStart();

        while (opModeIsActive()) {
            // Put run blocks here that get looped
            mecanumWheels.drive(this);
            // Update telemetry
            telemetry.update();
        }
    }

    /**
     * Constrain a double value between a minimum and maximum value.
     *
     * @param Var The value to constrain.
     * @param Min The minimum value.
     * @param Max The maximum value.
     * @return The constrained value.
     */
    public double constrain(double Var, double Min, double Max) {
        Var = Math.min(Math.max(Var, Min), Max);
        return Var;
    }

    /**
     * Constrain an integer value between a minimum and maximum value.
     *
     * @param Var The value to constrain.
     * @param Min The minimum value.
     * @param Max The maximum value.
     * @return The constrained value.
     */
    public int constrain(int Var, int Min, int Max) {
        Var = Math.min(Math.max(Var, Min), Max);
        return Var;
    }
}