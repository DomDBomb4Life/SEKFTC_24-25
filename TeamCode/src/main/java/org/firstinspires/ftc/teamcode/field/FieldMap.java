// File: FieldMap.java
package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.HashMap;
import java.util.Map;

/**
 * Represents the field and static objects for autonomous navigation.
 */
public class FieldMap {

    // Define static positions on the field
    public static final Pose2d STARTING_POSITION = new Pose2d(0, 0, Math.toRadians(0));

    // Map of AprilTag IDs to their poses on the field
    private static final Map<Integer, Pose2d> aprilTagPoses = new HashMap<>();

    static {
        // Initialize tag positions (example values)
        aprilTagPoses.put(1, new Pose2d(72, 36, Math.toRadians(0)));
        aprilTagPoses.put(2, new Pose2d(72, -36, Math.toRadians(0)));
        aprilTagPoses.put(3, new Pose2d(0, -72, Math.toRadians(90)));
        // Add more tags as needed
    }

    // Method to get the pose of an AprilTag by its ID
    public static Pose2d getAprilTagPose(int id) {
        return aprilTagPoses.get(id);
    }

    // Constructor
    private FieldMap() {
        // Prevent instantiation
    }
}