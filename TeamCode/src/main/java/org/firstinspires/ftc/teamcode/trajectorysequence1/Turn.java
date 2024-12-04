// File: Turn.java
package org.firstinspires.ftc.teamcode.trajectorysequence1;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Class representing a turn in place.
 */
public class Turn {
    private final Pose2d startPose;
    private final double angle;
    private final double maxAngVel;
    private final double maxAngAccel;

    public Turn(Pose2d startPose, double angle, double maxAngVel, double maxAngAccel) {
        this.startPose = startPose;
        this.angle = angle;
        this.maxAngVel = maxAngVel;
        this.maxAngAccel = maxAngAccel;
    }

    public Pose2d endPose() {
        return new Pose2d(
                startPose.getX(),
                startPose.getY(),
                startPose.getHeading() + angle
        );
    }

    public double getAngle() {
        return angle;
    }

    public double getMaxAngVel() {
        return maxAngVel;
    }

    public double getMaxAngAccel() {
        return maxAngAccel;
    }
}