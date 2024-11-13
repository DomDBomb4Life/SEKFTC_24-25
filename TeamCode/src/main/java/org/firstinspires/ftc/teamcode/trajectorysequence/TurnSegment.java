package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.ArrayList;
import java.util.List;

/**
* Segment representing a turn in place.
*/
public class TurnSegment extends SequenceSegment {
   private final double totalRotation;
   private final MotionProfile motionProfile;
   private final List<TrajectoryMarker> markers;

   public TurnSegment(Pose2d startPose, double totalRotation, MotionProfile motionProfile, List<TrajectoryMarker> markers) {
       super(motionProfile.duration(), startPose, new Pose2d(startPose.vec(), startPose.getHeading() + totalRotation));
       this.totalRotation = totalRotation;
       this.motionProfile = motionProfile;
       this.markers = new ArrayList<>(markers);
   }

   public double getTotalRotation() {
       return totalRotation;
   }

   public MotionProfile getMotionProfile() {
       return motionProfile;
   }

   public List<TrajectoryMarker> getMarkers() {
       return markers;
   }

   @Override
   public SequenceSegment withMarker(TrajectoryMarker marker) {
       List<TrajectoryMarker> updatedMarkers = new ArrayList<>(markers);
       updatedMarkers.add(marker);
       return new TurnSegment(getStartPose(), totalRotation, motionProfile, updatedMarkers);
   }
}