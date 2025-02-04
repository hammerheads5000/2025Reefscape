// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */

public class AlignToReefCommands {
    private static final Translation2d REEF_CENTER_BLUE = APRIL_TAGS.getTagPose(18).get().toPose2d().getTranslation()
        .plus(APRIL_TAGS.getTagPose(21).get().toPose2d().getTranslation()).div(2); // pos between opposite tags

    private static final Translation2d REEF_CENTER_RED = APRIL_TAGS.getTagPose(10).get().toPose2d().getTranslation()
        .plus(APRIL_TAGS.getTagPose(21).get().toPose2d().getTranslation()).div(2); // pos between opposite tags

    // distance from center of robot to center of reef
    private static final Distance REEF_APOTHEM = Meters.of(
            APRIL_TAGS.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER_BLUE))
            .plus(DISTANCE_TO_REEF);

    // translation to move from centered on a side to scoring position for the left branch
    private static final Translation2d CENTERED_TO_LEFT_BRANCH = new Translation2d(Meters.of(0),
            Inches.of(12.94 / 2));

    /**
     * Calculates the pose of the robot for scoring on a branch or trough.
     *
     * @param side The side of the reef (0 for left, increases clockwise).
     * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
     * @return The calculated Pose2d for scoring.
     */
    public static Pose2d getReefPose(int side, int relativePos) {
        Translation2d reefCenter = DriverStation.getAlliance().get() == Alliance.Blue ? REEF_CENTER_BLUE : REEF_CENTER_RED;

        // robot position centered on left reef side
        Translation2d translation = reefCenter.plus(new Translation2d(REEF_APOTHEM.unaryMinus(), Meters.of(0)));
        // translate to correct branch (left, right, center)
        translation = translation.plus(CENTERED_TO_LEFT_BRANCH.times(relativePos));
        // rotate to correct side
        translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side));

        // give heading
        return new Pose2d(translation,
                Rotation2d.fromDegrees(-60 * side));
    }

    public static Command alignToReef(int side, int relativePos, Swerve swerve) {
        return new AlignToPoseCommand(getReefPose(side, relativePos), SCORING_PID_X, SCORING_PID_Y, SCORING_PID_ANGLE, swerve);
    }

    public void testReefPoses() {
        StructArrayPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructArrayTopic("Pts", Pose2d.struct).publish();
        Pose2d[] poses = new Pose2d[18];
        for (int side = 0; side < 6; side++) {
            Pose2d left = AlignToReefCommands.getReefPose(side, -1);
            Pose2d center = AlignToReefCommands.getReefPose(side, 0);
            Pose2d right = AlignToReefCommands.getReefPose(side, 1);
            poses[side*3] = left;
            poses[side*3+1] = center;
            poses[side*3+2] = right;
        }

        publisher.set(poses);
    }
}
