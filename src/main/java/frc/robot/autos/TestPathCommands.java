// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.AutoConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class TestPathCommands {
    public static Command moveForwardTest() {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(Meters.of(1), Meters.zero(), Rotation2d.fromDegrees(0)));

        PathPlannerPath path = new PathPlannerPath(
                waypoints, CONSTRAINTS, null,
                new GoalEndState(MetersPerSecond.of(0), Rotation2d.fromDegrees(0)));

        return AutoBuilder.followPath(path);
    }

    public static Command moveForwardBackTest() {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(Meters.of(1), Meters.zero(), Rotation2d.fromDegrees(0)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(180)));

        PathPlannerPath path = new PathPlannerPath(
                waypoints, CONSTRAINTS, null,
                new GoalEndState(MetersPerSecond.of(0), Rotation2d.fromDegrees(0)));

        return AutoBuilder.followPath(path);
    }

    public static Command moveSquareTest() {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(Meters.of(1), Meters.zero(), Rotation2d.fromDegrees(0)),
                new Pose2d(Meters.of(1), Meters.of(1), Rotation2d.fromDegrees(90)),
                new Pose2d(Meters.zero(), Meters.of(1), Rotation2d.fromDegrees(180)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(-90)));

        PathPlannerPath path = new PathPlannerPath(
                waypoints, CONSTRAINTS, null,
                new GoalEndState(MetersPerSecond.of(0), Rotation2d.fromDegrees(0)));

        return AutoBuilder.followPath(path);
    }
}
