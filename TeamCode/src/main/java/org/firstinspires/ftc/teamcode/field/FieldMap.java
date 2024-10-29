// File: FieldMap.java
package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Represents the field and static objects for autonomous navigation.
 */
public class FieldMap {

    // Define static positions on the field
    public static final Pose2d STARTING_POSITION = new Pose2d(0, 0, Math.toRadians(0));

    public static final Pose2d ALLIANCE_PIECE_1 = new Pose2d(24, 24, Math.toRadians(0));
    public static final Pose2d ALLIANCE_PIECE_2 = new Pose2d(48, 24, Math.toRadians(0));
    public static final Pose2d ALLIANCE_PIECE_3 = new Pose2d(72, 24, Math.toRadians(0));

    public static final Pose2d LOW_RUNG_POSITION = new Pose2d(36, 0, Math.toRadians(90));

    // Add other static field elements as needed

    // Constructor
    private FieldMap() {
        // Prevent instantiation
    }
}