package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.ArrayList;
import java.util.List;

/**
* Segment representing a trajectory.
*/
public class TrajectorySegment extends SequenceSegment {
   private final Trajectory trajectory;

   public TrajectorySegment(Trajectory trajectory) {
       super(trajectory.duration(), trajectory.start(), trajectory.end());
       this.trajectory = trajectory;
   }

   public Trajectory getTrajectory() {
       return trajectory;
   }

   @Override
   public SequenceSegment withMarker(TrajectoryMarker marker) {
       // Add the marker to the trajectory
       List<TrajectoryMarker> newMarkers = new ArrayList<>(trajectory.getMarkers());
       newMarkers.add(marker);

       // Create a new Trajectory with updated markers
       Trajectory updatedTrajectory = new Trajectory(
               trajectory.getPath(),
               trajectory.getProfile(),
               newMarkers
       );

       return new TrajectorySegment(updatedTrajectory);
   }
}