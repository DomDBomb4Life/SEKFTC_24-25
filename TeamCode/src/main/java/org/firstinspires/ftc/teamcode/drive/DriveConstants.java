// File: DriveConstants.java
package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Constants shared between multiple drive types.
 *
 * These constants should be tuned to match your robot's physical properties.
 */
@Config
public class DriveConstants {
    /*
     * Motor constants that should be listed in your motor's datasheet.
     */
    public static final double TICKS_PER_REV = 537.6; // Example for GoBILDA 312 RPM motors
    public static final double MAX_RPM = 312;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present or an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * Physical constants of the robot.
     */
    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 8.622; // in

    /*
     * Feedforward parameters for the drive motors.
     * These values should be empirically determined.
     */
//    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kV = 0.01865;

    public static double kA = 0.00077;
    public static double kStatic = 0.09341;

    /*
     * PID coefficients for the robot's drive control.
     */
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    /*
     * Lateral multiplier to correct for drivetrain imperfections.
     */
    public static double LATERAL_MULTIPLIER = 1.0;

    /*
     * Drive constraints for trajectory generation.
     */
    public static double MAX_VEL = rpmToVelocity(MAX_RPM);
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    /*
     * Weights for drive power during teleop control.
     */
    public static double VX_WEIGHT = 1.0;
    public static double VY_WEIGHT = 1.0;
    public static double OMEGA_WEIGHT = 1.0;

    /*
     * Adjust the hub orientation to match your robot's configuration.
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    /*
     * Helper methods for unit conversions.
     */
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // See the FTC SDK documentation for details on this calculation.
        return 32767 / ticksPerSecond;
    }
}