// File: AimbotController.java
package org.firstinspires.ftc.teamcode.aimbot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.localization.Localization;
import org.firstinspires.ftc.teamcode.drive.DriveTrain;

/**
 * Controls the robot's aiming functionality towards a target pose.
 */
public class AimbotController {
    private final Localization localization;
    private final DriveTrain driveTrain;
    private Pose2d targetPose;
    private boolean isActive;

    public AimbotController(Localization localization, DriveTrain driveTrain) {
        this.localization = localization;
        this.driveTrain = driveTrain;
        this.isActive = false;
    }

    public void activate(Pose2d targetPose) {
        this.targetPose = targetPose;
        this.isActive = true;
    }

    public void deactivate() {
        this.isActive = false;
    }

    public void update() {
        if (isActive) {
            Pose2d currentPose = localization.getCurrentPose();

            // Calculate the required movement to align with the target
            Pose2d error = new Pose2d(
                targetPose.getX() - currentPose.getX(),
                targetPose.getY() - currentPose.getY(),
                targetPose.getHeading() - currentPose.getHeading()
            );

            // Simple proportional controller
            double kP = 0.5; // Tuning parameter
            double xPower = kP * error.getX();
            double yPower = kP * error.getY();
            double turnPower = kP * error.getHeading();

            // Normalize powers
            double maxPower = Math.max(Math.abs(xPower), Math.abs(yPower));
            if (maxPower > 1.0) {
                xPower /= maxPower;
                yPower /= maxPower;
            }

            // Command the drivetrain
            driveTrain.driveFieldCentric(yPower, xPower, turnPower, currentPose.getHeading());
        }
    }

    public boolean isActive() {
        return isActive;
    }
}