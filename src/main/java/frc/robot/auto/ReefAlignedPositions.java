// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.LimelightConstants;

/**
 * Utility classes to store every aligned position (grabbed from {@link AprilTagMap#calculateReefAlignedPosition(int, int)}).
 * Also finds the closest one to a given position.
 */
public final class ReefAlignedPositions {
    /** Coral distance added before the real lign up in meters. Can technically score from here if pushing into a coral. */
    public static final double CORAL_DISTANCE = 0.11 /* Coral diameter */;
    /** In meters. */
    public static final double REEF_ALIGN_RANGE = 2.0;

    // x-coordinates
    private static final double RED_MID_REEF_LINE = 13.0589;
    private static final double BLUE_MID_REEF_LINE = 4.4893;

    private static final Pose2d[][] RED_SIDE = {
        { // LEFT
/*  6 */    new Pose2d(new Translation2d(13.53856, 2.83527), Rotation2d.fromDegrees(120)),
/*  7 */    new Pose2d(new Translation2d(14.3305,  3.8459 ), Rotation2d.fromDegrees(180)),
/*  8 */    new Pose2d(new Translation2d(13.85033, 5.03653), Rotation2d.fromDegrees(-120)),
            // WARNING : DO NOT CHANGE THE ORDERS OF THESE TRANSLATIONS, THEY ARE ORDERED BY SIDE OF THE REEF
/*  9 */    new Pose2d(new Translation2d(12.57924, 5.21653), Rotation2d.fromDegrees(-60)),
/* 10 */    new Pose2d(new Translation2d(11.78731, 4.2059 ), Rotation2d.fromDegrees(0)),
/* 11 */    new Pose2d(new Translation2d(12.26747, 3.01527), Rotation2d.fromDegrees(60))
        },
        { // MIDDLE
/*  6 */    new Pose2d(new Translation2d(13.69445, 2.92527), Rotation2d.fromDegrees(120)),
/*  7 */    new Pose2d(new Translation2d(14.3305,  4.0259 ), Rotation2d.fromDegrees(180)),
/*  8 */    new Pose2d(new Translation2d(13.69445, 5.12653), Rotation2d.fromDegrees(-120)),
            // WARNING : DO NOT CHANGE THE ORDERS OF THESE TRANSLATIONS, THEY ARE ORDERED BY SIDE OF THE REEF
/*  9 */    new Pose2d(new Translation2d(12.42336, 5.12653), Rotation2d.fromDegrees(-60)),
/* 10 */    new Pose2d(new Translation2d(11.78731, 4.0259 ), Rotation2d.fromDegrees(0)),
/* 11 */    new Pose2d(new Translation2d(12.42336, 2.92527), Rotation2d.fromDegrees(60)),
        },
        { // RIGHT
/*  6 */    new Pose2d(new Translation2d(13.85033, 3.01527), Rotation2d.fromDegrees(120)),
/*  7 */    new Pose2d(new Translation2d(14.3305,  4.2059 ), Rotation2d.fromDegrees(180)),
/*  8 */    new Pose2d(new Translation2d(13.53856, 5.21653), Rotation2d.fromDegrees(-120)),
            // WARNING : DO NOT CHANGE THE ORDERS OF THESE TRANSLATIONS, THEY ARE ORDERED BY SIDE OF THE REEF
/*  9 */    new Pose2d(new Translation2d(12.26747, 5.03653), Rotation2d.fromDegrees(-60)),
/* 10 */    new Pose2d(new Translation2d(11.78731, 3.8459 ), Rotation2d.fromDegrees(0)),
/* 11 */    new Pose2d(new Translation2d(12.57924, 2.83527), Rotation2d.fromDegrees(60))
        }
    };

    private static final Pose2d[][] BLUE_SIDE = {
        { // LEFT
/* 17 */    new Pose2d(new Translation2d(3.69802, 3.01527), Rotation2d.fromDegrees(60)),
/* 18 */    new Pose2d(new Translation2d(3.2176,  4.2059 ), Rotation2d.fromDegrees(0)),
/* 19 */    new Pose2d(new Translation2d(4.00979, 5.21653), Rotation2d.fromDegrees(-60)),
            // WARNING : DO NOT CHANGE THE ORDERS OF THESE TRANSLATIONS, THEY ARE ORDERED BY SIDE OF THE REEF
/* 20 */    new Pose2d(new Translation2d(5.28062, 5.03653), Rotation2d.fromDegrees(-120)),
/* 21 */    new Pose2d(new Translation2d(5.76105, 3.8459 ), Rotation2d.fromDegrees(180)),
/* 22 */    new Pose2d(new Translation2d(4.96886, 2.83527), Rotation2d.fromDegrees(120))
        },
        { // MIDDLE
/* 17 */    new Pose2d(new Translation2d(3.85391, 2.92527), Rotation2d.fromDegrees(60)),
/* 18 */    new Pose2d(new Translation2d(3.2176,  4.0259 ), Rotation2d.fromDegrees(0)),
/* 19 */    new Pose2d(new Translation2d(3.85391, 5.12653), Rotation2d.fromDegrees(-60)),
            // WARNING : DO NOT CHANGE THE ORDERS OF THESE TRANSLATIONS, THEY ARE ORDERED BY SIDE OF THE REEF
/* 20 */    new Pose2d(new Translation2d(5.12474, 5.12653), Rotation2d.fromDegrees(-120)),
/* 21 */    new Pose2d(new Translation2d(5.76105, 4.0259 ), Rotation2d.fromDegrees(180)),
/* 22 */    new Pose2d(new Translation2d(5.12474, 2.92527), Rotation2d.fromDegrees(120))
        },
        { // RIGHT
/* 17 */    new Pose2d(new Translation2d(4.00979, 2.83527), Rotation2d.fromDegrees(60)),
/* 18 */    new Pose2d(new Translation2d(3.2176,  3.8459), Rotation2d.fromDegrees(0)),
/* 19 */    new Pose2d(new Translation2d(3.69802, 5.03653), Rotation2d.fromDegrees(-60)),
            // WARNING : DO NOT CHANGE THE ORDERS OF THESE TRANSLATIONS, THEY ARE ORDERED BY SIDE OF THE REEF
/* 20 */    new Pose2d(new Translation2d(4.96886, 5.21653), Rotation2d.fromDegrees(-120)),
/* 21 */    new Pose2d(new Translation2d(5.76105, 4.2059), Rotation2d.fromDegrees(180)),
/* 22 */    new Pose2d(new Translation2d(5.28062, 3.01527), Rotation2d.fromDegrees(120))
        }
    };

    /**
     * Gets the aligned position closest to the robot on its side of the reef midway line.
     * @param position - The position to calculate distance from (generally the robot).
     * @param direction - The direction to align, robot-relative while facing tag, left or right (-1, 0, 1).
     * @return The aligned Pose closest to the robot with that direction in front of the tag.
     * @apiNote Returns Pose2d.kZero if the closest distance is greater than {@link LimelightConstants.REEF_ALIGN_RANGE}.
     */
    public static Pose2d getClosestReefAlignedPosition(Translation2d position, int direction) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        System.out.println(alliance.isEmpty());

        Pose2d blue, red;
        blue = red = Pose2d.kZero;

        if (alliance.orElse(Alliance.Blue) == Alliance.Blue) {
            blue = findClosestPoseInArray(position, ReefAlignedPositions.BLUE_SIDE[direction + 1]);
        }

        if (alliance.orElse(Alliance.Red) == Alliance.Red) {
            red = findClosestPoseInArray(position, ReefAlignedPositions.RED_SIDE[direction + 1]);
        }

        if (blue == Pose2d.kZero) return red;
        if (red == Pose2d.kZero) return blue;

        return position.getDistance(blue.getTranslation()) < position.getDistance(red.getTranslation())
            ? blue : red;
    }

    /**
     * Whether or not a position is on the driver-side of the reef (can be used to determine whether to flip aligning).
     * @param position - The current position.
     * @return Whether to flip.
     */
    public static boolean isPositionDriverSideOfReef(Translation2d position) {
        return position.getX() <= ReefAlignedPositions.BLUE_MID_REEF_LINE
            || position.getX() >= ReefAlignedPositions.RED_MID_REEF_LINE;
    }

    /**
     * Helper that searches an array for the closest position on the robot's side of the reef midway line.
     * @param position - The position to calculate distance from (generally the robot).
     * @param direction - The direction to align, robot-relative while facing tag, left or right (-1, 0, 1).
     * @return The closest position.
     * @apiNote Returns Pose2d.kZero if the closest position is further than {@link LimelightConstants#REEF_ALIGN_RANGE}.
     */
    private static Pose2d findClosestPoseInArray(Translation2d position, Pose2d[] poses) {
        double distance = -1;
        double newDistance;
        Pose2d closestPose = Pose2d.kZero;

        // If driver-side, search the driver side aligning positions (indexes 0, 1, 2. otherwise, 3, 4, 5.)
        for (
            int i = (isPositionDriverSideOfReef(position) ? 0 : 3);
            i <=    (isPositionDriverSideOfReef(position) ? 2 : 5);
            i++
        ) {
            newDistance = position.getDistance(poses[i].getTranslation());
            if (newDistance < ReefAlignedPositions.REEF_ALIGN_RANGE && (newDistance < distance || distance == -1)) {
                distance = newDistance;
                closestPose = poses[i];
            }
        }

        return closestPose;
    }
}
