// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static frc.robot.Constants.INST;
import static frc.robot.Constants.AutoConstants.CONSTRAINTS;
import static frc.robot.Constants.FieldConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/** Container class for approaching/moving to coral station */
public class ApproachCoralStationCommands {
    /**
     * @param station 0 for right station, 1 for left from operator perspective
     * @param relativePos -1 for right, 0 for center, 1 for left from operator perspective
     * @return Station
     */
    public static Pose2d getStationPose(int station, int relativePos) {
        Pose2d pose = station == 1 ? STATION_1 : STATION_0;
        Translation2d translation = new Translation2d(STATION_APPROACH_DISTANCE, SIDE_STATION_OFFSET.times(relativePos));

        pose = pose.transformBy(new Transform2d(translation, Rotation2d.kZero));

        return pose;
    }

    /** Publishes station poses to NetworkTables */
    public static void testStationPoses() {
        Pose2d[] poses = new Pose2d[]{
            getStationPose(0, -1),
            getStationPose(0, 0),
            getStationPose(0, 1),
            getStationPose(1, -1),
            getStationPose(1, 0),
            getStationPose(1, 1)
        };
        INST.getStructArrayTopic("Station Poses", Pose2d.struct).publish().set(poses);
    }

    /**
     * Generate PathPlannerPath to the desired station, moving around the reef if necessary
     * @param station 0 for right station, 1 for left from operator perspective
     * @param relativePos -1 for right, 0 for center, 1 for left from operator perspective
     * @param swerve
     * @return
     */
    public static Command pathfindCommand(int station, int relativePos, Swerve swerve) {
        Pose2d target = getStationPose(station, relativePos);
        
        return AutoBuilder.pathfindToPoseFlipped(target, CONSTRAINTS, 0);
    }
}
