// File: FieldConstants.java
package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Constants representing field positions and coordinates.
 */
public class FieldConstants {
    // Field dimensions in inches (assuming standard FTC field)
    public static final double FIELD_SIZE = 144.0; // 12 feet x 12 feet field

    // Tile size in inches
    public static final double TILE_SIZE = 24.0;

    // Distance from robot to wall
    public static final double WALL_DISTANCE = 7.55;

    // Starting positions (defined for one side)
    public static final Pose2d LEFT_START = new Pose2d(1.5 * TILE_SIZE, -(2 * TILE_SIZE + WALL_DISTANCE), Math.toRadians(90)); // Facing positive X
    public static final Pose2d RIGHT_START = new Pose2d(-0.5 * TILE_SIZE, -(2 * TILE_SIZE + WALL_DISTANCE), Math.toRadians(90));

    // Net position
    public static final Pose2d NET_POSITION = new Pose2d(48, -48, Math.toRadians(135));

    // Neutral Sample positions (assuming they are placed along the Y-axis)
    public static final Pose2d SAMPLE_1_POSITION = new Pose2d(-24 - 1.25, 2.5 * TILE_SIZE + 10, Math.toRadians(90));
    public static final Pose2d SAMPLE_2_POSITION = new Pose2d(-24 - 1.25, 2.5 * TILE_SIZE, Math.toRadians(90));
    public static final Pose2d SAMPLE_3_POSITION = new Pose2d(-24 - 1.25, 2.5 * TILE_SIZE - 10, Math.toRadians(90));

    // Ascent zone position
    public static final Pose2d ASCENT_ZONE_POSITION = new Pose2d(0, 36, 0);

    // Adjustments for headings
    public static final double NET_HEADING = Math.toRadians(45); // Adjust as needed

    /**
     * Mirrors a pose over the Y-axis to get the equivalent position on the other side of the field.
     *
     * @param pose The original pose.
     * @return The mirrored pose.
     */
    public static Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(-pose.getX(), pose.getY(), -pose.getHeading());
    }

    /**
     * Returns the starting pose based on the team color and starting position.
     *
     * @param teamColor        The team color (RED or BLUE).
     * @param startingPosition The starting position (LEFT or RIGHT).
     * @return The starting pose.
     */
    public static Pose2d getStartingPose(TeamColor teamColor, StartingPosition startingPosition) {
        Pose2d pose;
        if (startingPosition == StartingPosition.LEFT) {
            pose = LEFT_START;
        } else {
            pose = RIGHT_START;
        }

        if (teamColor == TeamColor.RED) {
            pose = mirrorPoseOverX(pose);
            pose = adjustHeadingForRed(pose);
        }

        return pose;
    }

    /**
     * Mirrors a pose over the X-axis to get the equivalent position on the other side of the field.
     *
     * @param pose The original pose.
     * @return The mirrored pose.
     */
    public static Pose2d mirrorPoseOverX(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), -pose.getHeading());
    }

    /**
     * Adjusts the heading for the RED team if necessary.
     *
     * @param pose The pose with heading to adjust.
     * @return The pose with adjusted heading.
     */
    private static Pose2d adjustHeadingForRed(Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading() + Math.toRadians(180));
    }

    /**
     * Returns the net position based on the team color.
     *
     * @param teamColor The team color (RED or BLUE).
     * @return The net position.
     */
    public static Pose2d getNetPosition(TeamColor teamColor) {
        Pose2d pose = NET_POSITION;

        if (teamColor == TeamColor.RED) {
            pose = mirrorPoseOverX(pose);
            pose = adjustHeadingForRed(pose);
        }

        return pose;
    }

    /**
     * Returns the sample positions based on the team color.
     *
     * @param teamColor The team color (RED or BLUE).
     * @return An array of sample positions.
     */
    public static Pose2d[] getSamplePositions(TeamColor teamColor) {
        Pose2d[] samples = new Pose2d[]{SAMPLE_1_POSITION, SAMPLE_2_POSITION, SAMPLE_3_POSITION};

        if (teamColor == TeamColor.RED) {
            for (int i = 0; i < samples.length; i++) {
                samples[i] = mirrorPoseOverX(samples[i]);
                samples[i] = adjustHeadingForRed(samples[i]);
            }
        }

        return samples;
    }

    /**
     * Returns the ascent zone position based on the team color.
     *
     * @param teamColor The team color (RED or BLUE).
     * @return The ascent zone position.
     */
    public static Pose2d getAscentZonePosition(TeamColor teamColor) {
        Pose2d pose = ASCENT_ZONE_POSITION;

        if (teamColor == TeamColor.RED) {
            pose = mirrorPoseOverX(pose);
            pose = adjustHeadingForRed(pose);
        }

        return pose;
    }

    // Enums for team color and starting position
    public enum TeamColor {
        RED,
        BLUE
    }

    public enum StartingPosition {
        LEFT,
        RIGHT
    }
}