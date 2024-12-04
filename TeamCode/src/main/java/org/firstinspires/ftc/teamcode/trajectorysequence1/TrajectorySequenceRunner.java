// File: TrajectorySequenceRunner.java
package org.firstinspires.ftc.teamcode.trajectorysequence1;

import com.acmerobotics.roadrunner.control.*;
import com.acmerobotics.roadrunner.drive.*;
import com.acmerobotics.roadrunner.followers.*;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.*;

/**
 * Class responsible for executing trajectory sequences.
 */
public class TrajectorySequenceRunner {
    private final TrajectoryFollower follower;
    private final PIDFController turnController;
    private final NanoClock clock;

    private TrajectorySequence currentSequence;
    private int currentSegmentIndex;
    private double segmentStartTime;

    private boolean isSegmentActive;

    public TrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPID) {
        this.follower = follower;
        this.turnController = new PIDFController(headingPID);
        this.clock = NanoClock.system();
        this.turnController.setInputBounds(0, 2 * Math.PI);

        this.isSegmentActive = false;
    }

    public void followTrajectorySequenceAsync(TrajectorySequence sequence) {
        this.currentSequence = sequence;
        this.currentSegmentIndex = 0;
        this.segmentStartTime = clock.seconds();
        this.isSegmentActive = false;
    }

    public DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity) {
        if (currentSequence == null) {
            return new DriveSignal();
        }

        if (currentSegmentIndex >= getTotalSegments()) {
            // Sequence complete
            currentSequence = null;
            return new DriveSignal();
        }

        Object currentSegment = getCurrentSegment();

        if (currentSegment instanceof Trajectory) {
            // Handle trajectory segments
            Trajectory trajectory = (Trajectory) currentSegment;

            if (!isSegmentActive) {
                follower.followTrajectory(trajectory);
                isSegmentActive = true;
            }

            DriveSignal signal = follower.update(poseEstimate, poseVelocity);

            if (!follower.isFollowing()) {
                // Move to the next segment
                currentSegmentIndex++;
                isSegmentActive = false;
            }

            return signal;
        } else if (currentSegment instanceof Turn) {
            // Handle turn segments
            Turn turn = (Turn) currentSegment;

            if (!isSegmentActive) {
                turnController.setTargetPosition(turn.endPose().getHeading());
                isSegmentActive = true;
            }

            double correction = turnController.update(poseEstimate.getHeading());

            if (Math.abs(Angle.normDelta(poseEstimate.getHeading() - turn.endPose().getHeading())) < Math.toRadians(1)) {
                // Turn complete
                currentSegmentIndex++;
                isSegmentActive = false;
            }

            return new DriveSignal(new Pose2d(0, 0, correction), new Pose2d());
        } else if (currentSegment instanceof Wait) {
            // Handle wait segments
            Wait wait = (Wait) currentSegment;

            double elapsedTime = clock.seconds() - segmentStartTime;

            if (elapsedTime >= wait.getDuration()) {
                // Wait complete
                currentSegmentIndex++;
                segmentStartTime = clock.seconds(); // Reset time for the next segment
            }

            return new DriveSignal(); // No movement during wait
        } else {
            throw new IllegalStateException("Unknown segment type in TrajectorySequenceRunner");
        }
    }

    public boolean isBusy() {
        return currentSequence != null;
    }

    public void stop() {
        currentSequence = null;
        isSegmentActive = false;
    }

    // Helper methods
    private int getTotalSegments() {
        return currentSequence.getTotalSegments();
    }

    private Object getCurrentSegment() {
        return currentSequence.getSegment(currentSegmentIndex);
    }
}