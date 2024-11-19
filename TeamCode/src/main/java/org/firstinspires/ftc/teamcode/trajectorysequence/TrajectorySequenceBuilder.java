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

    // Added variables for tangent handling
    private boolean setAbsoluteTangent = false;
    private double absoluteTangent = 0.0;
    private double tangentOffset = 0.0;

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

    // Method to set the starting tangent for the next trajectory
    public TrajectorySequenceBuilder setTangent(double tangent) {
        setAbsoluteTangent = true;
        absoluteTangent = tangent;
        return this;
    }

    // Method to set the trajectory to be reversed
    public TrajectorySequenceBuilder setReversed(boolean reversed) {
        tangentOffset = reversed ? Math.toRadians(180) : 0.0;
        setAbsoluteTangent = false;
        return this;
    }

    // Simplified trajectory methods

    // Line to a position, adjusting heading smoothly
    public TrajectorySequenceBuilder lineTo(Vector2d endPosition) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .lineTo(endPosition)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Line to a position, maintaining current heading
    public TrajectorySequenceBuilder lineToConstantHeading(Vector2d endPosition) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .lineToConstantHeading(endPosition)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Line to a pose with linear heading interpolation
    public TrajectorySequenceBuilder lineToLinearHeading(Pose2d endPose) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .lineToLinearHeading(endPose)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Line to a pose with spline heading interpolation
    public TrajectorySequenceBuilder lineToSplineHeading(Pose2d endPose) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .lineToSplineHeading(endPose)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Spline to a position with specified end heading
    public TrajectorySequenceBuilder splineTo(Vector2d endPosition, double endHeading) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .splineTo(endPosition, endHeading)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Spline to a position with constant heading
    public TrajectorySequenceBuilder splineToConstantHeading(Vector2d endPosition, double endHeading) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .splineToConstantHeading(endPosition, endHeading)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Spline to a pose with linear heading interpolation
    public TrajectorySequenceBuilder splineToLinearHeading(Pose2d endPose, double endHeading) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .splineToLinearHeading(endPose, endHeading)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Spline to a pose with spline heading interpolation
    public TrajectorySequenceBuilder splineToSplineHeading(Pose2d endPose, double endHeading) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .splineToSplineHeading(endPose, endHeading)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Strafe to a position
    public TrajectorySequenceBuilder strafeTo(Vector2d endPosition) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .strafeTo(endPosition)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Forward movement
    public TrajectorySequenceBuilder forward(double distance) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .forward(distance)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Backward movement
    public TrajectorySequenceBuilder back(double distance) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .back(distance)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Strafe left
    public TrajectorySequenceBuilder strafeLeft(double distance) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .strafeLeft(distance)
                .build();
        addTrajectorySegment(trajectory);
        return this;
    }

    // Strafe right
    public TrajectorySequenceBuilder strafeRight(double distance) {
        double startTangent = calculateStartTangent();
        Trajectory trajectory = new TrajectoryBuilder(
                currentPose,
                startTangent,
                currentVelConstraint,
                currentAccelConstraint
        )
                .strafeRight(distance)
                .build();
        addTrajectorySegment(trajectory);
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

    // Helper method to calculate the starting tangent
    private double calculateStartTangent() {
        return setAbsoluteTangent ? absoluteTangent : currentPose.getHeading() + tangentOffset;
    }

    // Helper method to add a trajectory segment and update current pose
    private void addTrajectorySegment(Trajectory trajectory) {
        sequenceSegments.add(trajectory);
        currentPose = trajectory.end();
        // Reset tangent settings after each segment
        setAbsoluteTangent = false;
        tangentOffset = 0.0;
    }
}