// File: TrajectorySequence.java
package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.List;

/**
 * Class representing a sequence of trajectories, turns, and waits.
 */
public class TrajectorySequence {
    private final List<Object> segments;
    private final Pose2d startPose;

    public TrajectorySequence(
            List<Object> segments,
            Pose2d startPose
    ) {
        this.segments = segments;
        this.startPose = startPose;
    }

    public int getTotalSegments() {
        return segments.size();
    }

    public Object getSegment(int index) {
        return segments.get(index);
    }

    public Pose2d getStartPose() {
        return startPose;
    }
}