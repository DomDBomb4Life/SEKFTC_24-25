// File: FieldMap.java
package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.HashMap;
import java.util.Map;

/**
 * Represents the field and static objects for autonomous navigation.
 * Includes AprilTag positions and other key locations.
 */
public class FieldMap {

    // Define static positions on the field
    public static final Pose2d STARTING_POSITION = new Pose2d(
            new Vector2d(0, 0), // Starting at origin
            0 // Facing forward
    );

    // Define various waypoints or positions on the field
    public static final Pose2d ALLIANCE_SAMPLE_1 = new Pose2d(new Vector2d(24, 0), 0);
    public static final Pose2d ALLIANCE_SAMPLE_2 = new Pose2d(new Vector2d(48, 0), 0);
    public static final Pose2d ALLIANCE_SAMPLE_3 = new Pose2d(new Vector2d(72, 0), 0);

    public static final Pose2d OBSERVATION_AREA = new Pose2d(new Vector2d(96, 0), Math.toRadians(90));

    public static final Pose2d YELLOW_SAMPLE_1 = new Pose2d(new Vector2d(120, -24), Math.toRadians(180));
    public static final Pose2d YELLOW_SAMPLE_2 = new Pose2d(new Vector2d(120, 24), Math.toRadians(180));

    public static final Pose2d BASKET_POSITION = new Pose2d(new Vector2d(144, 0), Math.toRadians(270));

    public static final Pose2d LOW_RUNG_POSITION = new Pose2d(new Vector2d(168, 0), Math.toRadians(0));

    // Map of AprilTag IDs to their poses on the field
    private static final Map<Integer, Pose2d> aprilTagPoses = new HashMap<>();

    static {
        // Initialize tag positions (example values)
        aprilTagPoses.put(1, new Pose2d(new Vector2d(0, 72), Math.toRadians(0)));
        aprilTagPoses.put(2, new Pose2d(new Vector2d(0, -72), Math.toRadians(0)));
        aprilTagPoses.put(3, new Pose2d(new Vector2d(72, 72), Math.toRadians(90)));
        aprilTagPoses.put(4, new Pose2d(new Vector2d(72, -72), Math.toRadians(90)));
        // Add more tags as needed
    }

    /**
     * Gets the pose of an AprilTag by its ID.
     *
     * @param id The ID of the AprilTag.
     * @return The Pose2d of the tag on the field.
     */
    public static Pose2d getAprilTagPose(int id) {
        return aprilTagPoses.get(id);
    }

    // Private constructor to prevent instantiation
    private FieldMap() {
    }
}