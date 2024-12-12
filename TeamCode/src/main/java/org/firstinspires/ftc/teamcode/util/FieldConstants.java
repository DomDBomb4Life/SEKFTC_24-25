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
    public static final double WALL_DISTANCE = 7.55+2.0;

    // Starting positions (defined for one side)
    public static final Pose2d LEFT_START = new Pose2d(1.5 * TILE_SIZE, (2 * TILE_SIZE + WALL_DISTANCE), Math.toRadians(270)); // Facing positive X
    public static final Pose2d RIGHT_START = new Pose2d(-0.5 * TILE_SIZE, (2 * TILE_SIZE + WALL_DISTANCE), Math.toRadians(270));

    // Net position
    public static final Pose2d NET_POSITION = new Pose2d(58, 55, Math.toRadians(225));

    // Neutral Sample positions (assuming they are placed along the Y-axis)
    public static final Pose2d SAMPLE_1_POSITION = new Pose2d(50, 41.8, Math.toRadians(270));
    public static final Pose2d SAMPLE_2_POSITION = new Pose2d(60, 41.8, Math.toRadians(270));
    public static final Pose2d SAMPLE_3_POSITION = new Pose2d(2.5 * TILE_SIZE + 10, 44, Math.toRadians(270));

    public static final Pose2d SAMPLE_4_POSITION = new Pose2d(-50, 41.8, Math.toRadians(270));
    public static final Pose2d SAMPLE_5_POSITION = new Pose2d(-60, 41.8, Math.toRadians(270));
    public static final Pose2d SAMPLE_6_POSITION = new Pose2d(2.5 * TILE_SIZE + 10, 44, Math.toRadians(270));

    public static final Pose2d SPECIMEN_SCORING_POSITION = new Pose2d(0, 40, Math.toRadians(270));



    // Ascent zone position
    public static final Pose2d ASCENT_ZONE_POSITION = new Pose2d(1.5*TILE_SIZE, 0, Math.toRadians(180));

    public static Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(-pose.getX(), pose.getY(), -pose.getHeading());
    }


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


    public static Pose2d mirrorPoseOverX(Pose2d pose) {
        return new Pose2d(pose.getX(), -pose.getY(), -pose.getHeading());
    }


    private static Pose2d adjustHeadingForRed(Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), pose.getHeading() + Math.toRadians(180));
    }


    public static Pose2d getNetPosition(TeamColor teamColor) {
        Pose2d pose = NET_POSITION;

        if (teamColor == TeamColor.RED) {
            pose = mirrorPoseOverX(pose);
            pose = adjustHeadingForRed(pose);
        }

        return pose;
    }


    public static Pose2d[] getSamplePositions(TeamColor teamColor) {
        Pose2d[] samples = new Pose2d[]{SAMPLE_1_POSITION, SAMPLE_2_POSITION
//                , SAMPLE_3_POSITION
        };

        if (teamColor == TeamColor.RED) {
            for (int i = 0; i < samples.length; i++) {
                samples[i] = mirrorPoseOverX(samples[i]);
                samples[i] = adjustHeadingForRed(samples[i]);
            }
        }

        return samples;
    }

    public static Pose2d[] getAllianceSamplePositions(TeamColor teamColor) {
        Pose2d[] samples = new Pose2d[]{SAMPLE_4_POSITION, SAMPLE_5_POSITION
        //      , SAMPLE_6_POSITION
        };
        return samples;
    }

    public static Pose2d getAscentZonePosition(TeamColor teamColor) {
        Pose2d pose = ASCENT_ZONE_POSITION;

        if (teamColor == TeamColor.RED) {
            pose = mirrorPoseOverX(pose);
            pose = adjustHeadingForRed(pose);
        }

        return pose;
    }
    public static Pose2d getSpecimenScoringPosition(TeamColor teamColor) {
        Pose2d pose = SPECIMEN_SCORING_POSITION;

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