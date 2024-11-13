package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.ArrayList;
import java.util.List;

/**
* Segment representing a wait (pause) at a specific pose.
*/
public class WaitSegment extends SequenceSegment {
   private final List<TrajectoryMarker> markers;

   public WaitSegment(Pose2d pose, double duration, List<TrajectoryMarker> markers) {
       super(duration, pose, pose);
       this.markers = new ArrayList<>(markers);
   }

   public List<TrajectoryMarker> getMarkers() {
       return markers;
   }

   @Override
   public SequenceSegment withMarker(TrajectoryMarker marker) {
       List<TrajectoryMarker> updatedMarkers = new ArrayList<>(markers);
       updatedMarkers.add(marker);
       return new WaitSegment(getStartPose(), getDuration(), updatedMarkers);
   }
}