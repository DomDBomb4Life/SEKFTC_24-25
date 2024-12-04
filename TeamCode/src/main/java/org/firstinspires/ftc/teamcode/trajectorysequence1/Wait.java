// File: Wait.java
package org.firstinspires.ftc.teamcode.trajectorysequence1;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Class representing a wait (pause) at a specific pose.
 */
public class Wait {
    private final Pose2d pose;
    private final double duration;

    public Wait(Pose2d pose, double duration) {
        this.pose = pose;
        this.duration = duration;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getDuration() {
        return duration;
    }
}