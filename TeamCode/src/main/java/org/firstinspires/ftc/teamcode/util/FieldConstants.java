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
    public static final Pose2d LEFT_START = new Pose2d(7,57.8 , Math.toRadians(270));
    public static final Pose2d RIGHT_START = new Pose2d(-8,57.8, Math.toRadians(270));

    // Key field targets
    public static final Pose2d NET_POSITION = new Pose2d(59.1, 55.3, Math.toRadians(225));


    public static final Pose2d ASCENT_ZONE_POSITION = new Pose2d(29.2, 1.3, Math.toRadians(180));

    public static final Pose2d SPECIMEN_SCORING_POSITION = new Pose2d(-8,36.4 , Math.toRadians(270));

    public static final Pose2d SPECIMEN_SCORING_POSITION_LEFT = new Pose2d(6.9,36.4 , Math.toRadians(270));

    // Sample positions
    public static final Pose2d[] SAMPLE_POSITIONS = {
            new Pose2d(47.5, 46.2, Math.toRadians(270)),
            new Pose2d(56.6, 46.1, Math.toRadians(270)),
            new Pose2d(54.32,42.34, Math.toRadians(303.84))
    };
}