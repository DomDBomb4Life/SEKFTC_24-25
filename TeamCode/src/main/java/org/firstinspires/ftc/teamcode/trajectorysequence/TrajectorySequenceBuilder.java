// File: TrajectorySequenceBuilder.java
package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Builder class for creating a trajectory sequence.
 */
public class TrajectorySequenceBuilder {
    private final Pose2d startPose;
    private final TrajectoryVelocityConstraint baseVelConstraint;
    private final TrajectoryAccelerationConstraint baseAccelConstraint;
    private final double baseMaxAngVel;
    private final double baseMaxAngAccel;

    private TrajectoryVelocityConstraint currentVelConstraint;
    private TrajectoryAccelerationConstraint currentAccelConstraint;
    private double currentMaxAngVel;
    private double currentMaxAngAccel;

    private List<Object> sequenceSegments; // Stores trajectories, turns, and waits

    private Pose2d currentPose;

    // Constructor
    public TrajectorySequenceBuilder(
            Pose2d startPose,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseMaxAngVel,
            double baseMaxAngAccel
    ) {
        this.startPose = startPose;
        this.baseVelConstraint = baseVelConstraint;
        this.baseAccelConstraint = baseAccelConstraint;
        this.baseMaxAngVel = baseMaxAngVel;
        this.baseMaxAngAccel = baseMaxAngAccel;

        this.currentVelConstraint = baseVelConstraint;
        this.currentAccelConstraint = baseAccelConstraint;
        this.currentMaxAngVel = baseMaxAngVel;
        this.currentMaxAngAccel = baseMaxAngAccel;

        this.sequenceSegments = new ArrayList<>();

        this.currentPose = startPose;
    }

    // Simplified trajectory methods
    public TrajectorySequenceBuilder lineTo(Vector2d endPosition) {
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                currentVelConstraint,
                currentAccelConstraint
        )
                .lineTo(endPosition)
                .build();
        sequenceSegments.add(trajectory);
        currentPose = trajectory.end();
        return this;
    }

    public TrajectorySequenceBuilder splineTo(Vector2d endPosition, double endHeading) {
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                currentVelConstraint,
                currentAccelConstraint
        )
                .splineTo(endPosition, endHeading)
                .build();
        sequenceSegments.add(trajectory);
        currentPose = trajectory.end();
        return this;
    }

    // Added methods
    public TrajectorySequenceBuilder forward(double distance) {
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                currentVelConstraint,
                currentAccelConstraint
        )
                .forward(distance)
                .build();
        sequenceSegments.add(trajectory);
        currentPose = trajectory.end();
        return this;
    }

    public TrajectorySequenceBuilder back(double distance) {
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                currentVelConstraint,
                currentAccelConstraint
        )
                .back(distance)
                .build();
        sequenceSegments.add(trajectory);
        currentPose = trajectory.end();
        return this;
    }

    public TrajectorySequenceBuilder strafeLeft(double distance) {
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                currentVelConstraint,
                currentAccelConstraint
        )
                .strafeLeft(distance)
                .build();
        sequenceSegments.add(trajectory);
        currentPose = trajectory.end();
        return this;
    }

    public TrajectorySequenceBuilder strafeRight(double distance) {
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                currentVelConstraint,
                currentAccelConstraint
        )
                .strafeRight(distance)
                .build();
        sequenceSegments.add(trajectory);
        currentPose = trajectory.end();
        return this;
    }

    // Turn method
    public TrajectorySequenceBuilder turn(double angle) {
        Turn turn = new Turn(currentPose, angle, currentMaxAngVel, currentMaxAngAccel);
        sequenceSegments.add(turn);
        currentPose = turn.endPose();
        return this;
    }

    // Wait method
    public TrajectorySequenceBuilder waitSeconds(double seconds) {
        Wait wait = new Wait(currentPose, seconds);
        sequenceSegments.add(wait);
        return this;
    }

    // Constraint setters
    public TrajectorySequenceBuilder setVelConstraint(TrajectoryVelocityConstraint velConstraint) {
        this.currentVelConstraint = velConstraint;
        return this;
    }

    public TrajectorySequenceBuilder resetVelConstraint() {
        this.currentVelConstraint = baseVelConstraint;
        return this;
    }

    public TrajectorySequenceBuilder setAccelConstraint(TrajectoryAccelerationConstraint accelConstraint) {
        this.currentAccelConstraint = accelConstraint;
        return this;
    }

    public TrajectorySequenceBuilder resetAccelConstraint() {
        this.currentAccelConstraint = baseAccelConstraint;
        return this;
    }

    public TrajectorySequenceBuilder setTurnConstraint(double maxAngVel, double maxAngAccel) {
        this.currentMaxAngVel = maxAngVel;
        this.currentMaxAngAccel = maxAngAccel;
        return this;
    }

    public TrajectorySequenceBuilder resetTurnConstraint() {
        this.currentMaxAngVel = baseMaxAngVel;
        this.currentMaxAngAccel = baseMaxAngAccel;
        return this;
    }

    // Build method
    public TrajectorySequence build() {
        return new TrajectorySequence(sequenceSegments, startPose);
    }
}