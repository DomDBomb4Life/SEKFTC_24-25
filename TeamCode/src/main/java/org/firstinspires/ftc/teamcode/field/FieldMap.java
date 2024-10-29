// File: FieldMap.java
package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Represents the field and static objects for autonomous navigation.
 */
public class FieldMap {

    // Define static positions on the field
    public static final Pose2d STARTING_POSITION = new Pose2d(0, 0, Math.toRadians(0));

    // Alliance sample positions (blue alliance)
    public static final Pose2d ALLIANCE_SAMPLE_1 = new Pose2d(12, 24, Math.toRadians(0));
    public static final Pose2d ALLIANCE_SAMPLE_2 = new Pose2d(24, 24, Math.toRadians(0));
    public static final Pose2d ALLIANCE_SAMPLE_3 = new Pose2d(36, 24, Math.toRadians(0));

    // Observation area position
    public static final Pose2d OBSERVATION_AREA = new Pose2d(60, 24, Math.toRadians(0));

    // Yellow sample positions
    public static final Pose2d YELLOW_SAMPLE_1 = new Pose2d(24, 48, Math.toRadians(0));
    public static final Pose2d YELLOW_SAMPLE_2 = new Pose2d(36, 48, Math.toRadians(0));

    // Basket positions
    public static final Pose2d BASKET_POSITION = new Pose2d(72, 48, Math.toRadians(0));

    // Low rung position for level 1 ascent
    public static final Pose2d LOW_RUNG_POSITION = new Pose2d(84, 24, Math.toRadians(90));

    // Constructor
    private FieldMap() {
        // Prevent instantiation
    }
}