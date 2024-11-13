package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.control.*;
import com.acmerobotics.roadrunner.drive.*;
import com.acmerobotics.roadrunner.followers.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

/**
 * Class responsible for executing trajectory sequences.
 */
public class TrajectorySequenceRunner {
    private final TrajectoryFollower follower;
    private final PIDFController turnController;
    private final VoltageSensor voltageSensor;
    private final NanoClock clock;

    private TrajectorySequence currentSequence;
    private double sequenceStartTime;
    private int currentSegmentIndex;

    // New field to track if we've started following the current segment
    private boolean segmentStarted = false;

    public TrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPID, VoltageSensor voltageSensor) {
        this.follower = follower;
        this.turnController = new PIDFController(headingPID);
        this.voltageSensor = voltageSensor;
        this.clock = NanoClock.system();
        this.turnController.setInputBounds(0, 2 * Math.PI);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence sequence) {
        this.currentSequence = sequence;
        this.sequenceStartTime = clock.seconds();
        this.currentSegmentIndex = 0;
        this.segmentStarted = false;
    }

    public DriveSignal update(Pose2d poseEstimate) {
        if (currentSequence == null) {
            return new DriveSignal();
        }

        double now = clock.seconds();
        double elapsedTime = now - sequenceStartTime;

        if (currentSegmentIndex >= currentSequence.size()) {
            currentSequence = null;
            return new DriveSignal();
        }

        SequenceSegment currentSegment = currentSequence.get(currentSegmentIndex);

        if (currentSegment instanceof TrajectorySegment) {
            TrajectorySegment trajSegment = (TrajectorySegment) currentSegment;

            // Ensure we start following the trajectory at the beginning of the segment
            if (!segmentStarted) {
                follower.followTrajectory(trajSegment.getTrajectory());
                segmentStarted = true;
            }

            DriveSignal signal = follower.update(poseEstimate);

            if (!follower.isFollowing()) {
                currentSegmentIndex++;
                sequenceStartTime = now;
                segmentStarted = false;
            }
            return signal;
        } else if (currentSegment instanceof TurnSegment) {
            TurnSegment turnSegment = (TurnSegment) currentSegment;
            MotionProfile profile = turnSegment.getMotionProfile();
            double profileTime = elapsedTime;
            if (profileTime > profile.duration()) {
                profileTime = profile.duration();
                currentSegmentIndex++;
                sequenceStartTime = now;
            }
            MotionState targetState = profile.get(profileTime);

            turnController.setTargetPosition(targetState.getX());
            double correction = turnController.update(poseEstimate.getHeading(), targetState.getV());

            return new DriveSignal(new Pose2d(0, 0, correction), new Pose2d(0, 0, 0));
        } else if (currentSegment instanceof WaitSegment) {
            WaitSegment waitSegment = (WaitSegment) currentSegment;
            if (elapsedTime >= waitSegment.getDuration()) {
                currentSegmentIndex++;
                sequenceStartTime = now;
            }
            return new DriveSignal();
        }

        return new DriveSignal();
    }

    public boolean isBusy() {
        return currentSequence != null;
    }

    public void stop() {
        currentSequence = null;
        segmentStarted = false;
    }
}