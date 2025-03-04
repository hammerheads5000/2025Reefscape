// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Swerve;

/**
 * Container class for aligning to reef     
 */
public class AlignToReefCommands {
    private static boolean flipToRed; // whether to use red reef (otherwise blue)

    /**
     * Calculates the pose of the robot for scoring on a branch or trough.
     *
     * @param side The side of the reef (0 for left, increases clockwise).
     * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
     * @return The calculated Pose2d for scoring.
     */
    public static Pose2d getReefPose(int side, int relativePos) {
        // determine whether to use red or blue reef position
        boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

        return getReefPose(side, relativePos, isRed);
    }

    /**
     * Calculates the pose of the robot for scoring on a branch or trough.
     *
     * @param side The side of the reef (0 for left, increases clockwise).
     * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
     * @return The calculated Pose2d for scoring.
     */
    public static Pose2d getReefPose(int side, int relativePos, boolean isRed) {
        // determine whether to use red or blue reef position
        flipToRed = isRed;

        // initially do all calculations from blue, then flip later
        Translation2d reefCenter = REEF_CENTER_BLUE;

        // robot position centered on close reef side
        Translation2d translation = reefCenter.plus(new Translation2d(REEF_APOTHEM.unaryMinus(), Meters.zero()));
        // translate to correct branch (left, right, center)
        translation = translation.plus(CENTERED_TO_LEFT_BRANCH.times(relativePos));
        // rotate to correct side
        translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side));

        // make pose from translation and correct rotation
        Pose2d reefPose = new Pose2d(translation,
                Rotation2d.fromDegrees(-60 * side));

        if (flipToRed) {
            reefPose = flipPose(reefPose);
        }

        return reefPose;
    }

    private static Pose2d flipPose(Pose2d pose) {
        Translation2d center = REEF_CENTER_BLUE.interpolate(REEF_CENTER_RED, 0.5);
        Translation2d poseTranslation = pose.getTranslation();
        poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
        return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    public static AlignToPoseCommand alignToReef(int side, int relativePos, Swerve swerve) {
        return new AlignToPoseCommand(getReefPose(side, relativePos), SCORING_PID_X, SCORING_PID_Y, SCORING_PID_ANGLE, swerve);
    }

    public static void testReefPoses() {
        testReefPoses(false, -1);
        testReefPoses(false, 0);
        testReefPoses(false, 1);
        testReefPoses(true, -1);
        testReefPoses(true, 0);
        testReefPoses(true, 1);
    }

    public static void testReefPoses(boolean isRed, int relativePos) {
        String topicName = "Reef Alignment Poses/";
        topicName += isRed ? "Red " : "Blue ";
        if (relativePos == -1) {
            topicName += "Right ";
        }
        else if (relativePos == 1) {
            topicName += "Left ";
        }
        else {
            topicName += "Center ";
        }
        
        StructArrayPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructArrayTopic(topicName, Pose2d.struct).publish();
        
        Pose2d[] poses = new Pose2d[6];
        for (int side = 0; side < 6; side++) {
            poses[side] = getReefPose(side, relativePos, isRed);
        }

        publisher.set(poses);
    }
}
