package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Simplified Field Constants:
 * - Removed team color adjustments and complexity.
 * - All positions are final static and directly editable.
 * - Sample positions stored in a static array.
 * - No methods to mirror or adjust poses.
 */

public class FieldConstants {

    // Basic field layout
    public static final double FIELD_SIZE = 144.0;
    public static final double TILE_SIZE = 24.0;
    public static final double WALL_DISTANCE = 7.55 + 2.5; // Additional offset as per previous code

    // Starting positions (just pick the final poses)
    public static final Pose2d LEFT_START = new Pose2d(39.8,57.8 , Math.toRadians(270));
    public static final Pose2d RIGHT_START = new Pose2d(-0.5 * TILE_SIZE, (2 * TILE_SIZE + WALL_DISTANCE), Math.toRadians(270));

    // Key field targets
    public static final Pose2d NET_POSITION = new Pose2d(59.1, 55.3, Math.toRadians(225));


    public static final Pose2d ASCENT_ZONE_POSITION = new Pose2d(29.2, 1.3, Math.toRadians(180));
    public static final Pose2d SPECIMEN_SCORING_POSITION = new Pose2d(36.1,0.3 , Math.toRadians(270));

    // Sample positions
    public static final Pose2d[] SAMPLE_POSITIONS = {
            new Pose2d(47.8, 45.9, Math.toRadians(270)),
            new Pose2d(58.2, 45.6, Math.toRadians(270))
    };
}