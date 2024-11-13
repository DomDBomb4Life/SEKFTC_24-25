package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

/**
* Abstract class representing a segment in a trajectory sequence.
*/
public abstract class SequenceSegment {
   private final double duration;
   private final Pose2d startPose;
   private final Pose2d endPose;

   protected SequenceSegment(double duration, Pose2d startPose, Pose2d endPose) {
       this.duration = duration;
       this.startPose = startPose;
       this.endPose = endPose;
   }

   public double getDuration() {
       return duration;
   }

   public Pose2d getStartPose() {
       return startPose;
   }

   public Pose2d getEndPose() {
       return endPose;
   }

   // Abstract method to be implemented by subclasses
   public abstract SequenceSegment withMarker(TrajectoryMarker marker);
}