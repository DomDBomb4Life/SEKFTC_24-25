// File: TrajectoryGenerator.java
package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

/**
 * Utility class for generating basic trajectories.
 */
public class TrajectoryGenerator {

    public static Trajectory generateLineTo(
            Pose2d startPose,
            Vector2d endPosition,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        TrajectoryBuilder builder = new TrajectoryBuilder(
                startPose,
                velConstraint,
                accelConstraint
        );
        builder.lineTo(endPosition);
        return builder.build();
    }

    public static Trajectory generateSplineTo(
            Pose2d startPose,
            Vector2d endPosition,
            double endHeading,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        TrajectoryBuilder builder = new TrajectoryBuilder(
                startPose,
                velConstraint,
                accelConstraint
        );
        builder.splineTo(endPosition, endHeading);
        return builder.build();
    }
}