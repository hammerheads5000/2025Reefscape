// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */

public class AlignToReefCommands {
    private static final Translation2d REEF_CENTER = APRIL_TAGS.getTagPose(18).get().toPose2d().getTranslation()
            .plus(APRIL_TAGS.getTagPose(21).get().toPose2d().getTranslation()).div(2); // pos between opposite tags

    // distance from center of robot to center of reef
    private static final Distance REEF_APOTHEM = Meters.of(
            APRIL_TAGS.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER))
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
    private static Pose2d getReefPose(int side, int relativePos) {
        // robot position centered on left reef side
        Translation2d centered = REEF_CENTER.plus(new Translation2d(REEF_APOTHEM.unaryMinus(), Meters.of(0)));
        // rotate to correct side
        centered = centered.rotateAround(REEF_CENTER, Rotation2d.fromDegrees(-60 * side));

        // translate centered to correct position (left, right, center) and give heading
        return new Pose2d(centered.plus(CENTERED_TO_LEFT_BRANCH.times(relativePos)),
                Rotation2d.fromDegrees(-60 * side));
    }

    public static Command alignToReef(int side, int relativePos) {
        return new AlignToPoseCommand(getReefPose(side, relativePos));
    }
}
