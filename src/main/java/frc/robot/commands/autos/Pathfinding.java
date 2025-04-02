// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.AutoConstants.APPROACH_CONSTRAINTS;
import static frc.robot.Constants.AutoConstants.APPROACH_DISTANCE;
import static frc.robot.Constants.AutoConstants.CONSTRAINTS;
import static frc.robot.Constants.AutoConstants.FAST_CONSTRAINTS;
import static frc.robot.Constants.AutoConstants.MIN_PATH_SPEED;
import static frc.robot.Constants.AutoConstants.TRAVERSE_DISTANCE;
import static frc.robot.Constants.FieldConstants.REEF_CENTER_BLUE;
import static frc.robot.Constants.FieldConstants.REEF_CENTER_RED;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.AlignToReefCommands;

/** Add your docs here. */
public class Pathfinding {
    public static int getClosestReefSide(Pose2d pose) {
        int closest = 0;
        double closestDistance = Double.POSITIVE_INFINITY;
        for (int side = 0; side < 6; side++) {
            Pose2d reefPose = AlignToReefCommands.getReefPose(side, 0);
            double distance = reefPose.getTranslation().getDistance(pose.getTranslation());
            if (distance < closestDistance) {
                closest = side;
                closestDistance = distance;
            }
        }

        return closest;
    }

    private static int distanceBetweenSides(int side1, int side2) {
        int diff = Math.abs(side1 - side2);
        return Math.min(diff, 6 - diff);
    }

    private static Pose2d pointPoseTowards(Pose2d pose, Translation2d other) {
        Translation2d translation = other.minus(pose.getTranslation());
        return new Pose2d(pose.getTranslation(), translation.getAngle());
    }

    private static Pose2d pointPoseTowards(Pose2d pose, Pose2d other) {
        return pointPoseTowards(pose, other.getTranslation());
    }

    private static int getNextSide(int currentSide, int targetSide) {
        int positiveDistance = distanceBetweenSides(currentSide + 1, targetSide);
        int negativeDistance = distanceBetweenSides(currentSide - 1, targetSide);

        if (positiveDistance < negativeDistance) return (currentSide + 1 + 6) % 6;
        return (currentSide - 1 + 6) % 6;
    }

    private static ArrayList<Pose2d> generateApproachPoses(Pose2d currentPose, int side) {
        int currentSide = getClosestReefSide(currentPose);
        
        ArrayList<Pose2d> poses = new ArrayList<>();
        while (distanceBetweenSides(currentSide, side) > 1) {
            int nextSide = getNextSide(currentSide, side);
            Pose2d sidePose = AlignToReefCommands.getReefPose(nextSide, 0);
            
            sidePose = sidePose.transformBy(new Transform2d(new Translation2d(TRAVERSE_DISTANCE.unaryMinus(), Meters.zero()), Rotation2d.kZero));
            
            if (nextSide == (currentSide + 1 + 6) % 6) {
                sidePose = sidePose.rotateAround(sidePose.getTranslation(), Rotation2d.kCCW_90deg);
            } else {
                sidePose = sidePose.rotateAround(sidePose.getTranslation(), Rotation2d.kCW_90deg);
            }

            poses.add(sidePose);
            currentSide = nextSide;
        }

        return poses;
    }

    public static PathPlannerPath generateReefPath(Pose2d currentPose, int side, double relativePos, ChassisSpeeds startSpeeds) {
        ArrayList<Pose2d> poses = generateApproachPoses(currentPose, side);
        Pose2d endPose = AlignToReefCommands.getReefPose(side, relativePos);
        Pose2d approachEndPose = AlignToReefCommands.getReefPose(side, relativePos);
        Transform2d shiftApproachTransform = new Transform2d(new Translation2d(APPROACH_DISTANCE.unaryMinus(), Meters.zero()), Rotation2d.kZero);
        approachEndPose = approachEndPose.transformBy(shiftApproachTransform);
        // if (poses.size() > 0) {
        //     poses.set(poses.size()-1, pointPoseTowards(poses.get(poses.size()-1), endPose)); 
        // }
        poses.add(approachEndPose);
        poses.add(endPose);

        Translation2d vel = new Translation2d(startSpeeds.vxMetersPerSecond, startSpeeds.vyMetersPerSecond);

        if (DriverStation.isAutonomous() || vel.getNorm() < MIN_PATH_SPEED.in(MetersPerSecond)) {
            poses.add(0, pointPoseTowards(currentPose, poses.get(0)));
        } else {
            poses.add(0, new Pose2d(currentPose.getTranslation(), chassisSpeedsToHeading(startSpeeds)));
        }

        ArrayList<RotationTarget> rotationTargets = new ArrayList<>();
        rotationTargets.add(new RotationTarget(poses.size()-2, endPose.getRotation()));

        ArrayList<PointTowardsZone> pointTowardsZones = new ArrayList<>();
        Translation2d reefCenter = AutoBuilder.shouldFlip() ? REEF_CENTER_RED : REEF_CENTER_BLUE;
        pointTowardsZones.add(new PointTowardsZone("Point At Reef", reefCenter, 0, poses.size()-2));

        ArrayList<ConstraintsZone> constraintsZones = new ArrayList<>();
        double slowAmount;
        
        constraintsZones.add(new ConstraintsZone(0, 0.5, FAST_CONSTRAINTS));
        constraintsZones.add(new ConstraintsZone(poses.size()-2, poses.size()-1, APPROACH_CONSTRAINTS));

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(poses),
                rotationTargets,
                pointTowardsZones,
                constraintsZones,
                new ArrayList<EventMarker>(),
                CONSTRAINTS,
                new IdealStartingState(chassisSpeedsToVelocity(startSpeeds), currentPose.getRotation()),
                new GoalEndState(0, endPose.getRotation()),
                false
        );

        if (AutoBuilder.shouldFlip()) {
            path = path.flipPath();            
        }
        return path;
    }

    public static PathPlannerPath generateReefPath(Pose2d currentPose, int side, int relativePos) {
        return generateReefPath(currentPose, side, relativePos, new ChassisSpeeds());
    }

    private static Rotation2d chassisSpeedsToHeading(ChassisSpeeds chassisSpeeds) {
        Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        return translation.getAngle();
    }

    private static LinearVelocity chassisSpeedsToVelocity(ChassisSpeeds chassisSpeeds) {
        Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        return MetersPerSecond.of(translation.getNorm());
    }
    
    public static PathPlannerPath generateStationPath(Pose2d currentPose, int station, int relativePos, ChassisSpeeds startSpeeds) {
        int side = station == 1 ? 1 : 5;
        ArrayList<Pose2d> poses = generateApproachPoses(currentPose, side);
        Pose2d endPose = ApproachCoralStationCommands.getStationPose(station, relativePos);
        endPose = endPose.rotateAround(endPose.getTranslation(), Rotation2d.k180deg);
        poses.add(endPose);
        
        Translation2d vel = new Translation2d(startSpeeds.vxMetersPerSecond, startSpeeds.vyMetersPerSecond);

        Pose2d startPose;
        if (DriverStation.isAutonomous() || vel.getNorm() < MIN_PATH_SPEED.in(MetersPerSecond)) {
            Pose2d closestReefSide = AlignToReefCommands.getReefPose(getClosestReefSide(currentPose), 0);
            if (closestReefSide.getTranslation().getDistance(currentPose.getTranslation()) < APPROACH_DISTANCE.in(Meters)) {
                startPose = new Pose2d(currentPose.getTranslation(), closestReefSide.getRotation().rotateBy(Rotation2d.k180deg));
            } else {
                startPose = pointPoseTowards(currentPose, poses.get(0));
            }
        } else {
            Rotation2d startHeading = chassisSpeedsToHeading(startSpeeds);
            startPose = new Pose2d(currentPose.getTranslation(), startHeading);
        }
        poses.add(0, startPose);
        
        ArrayList<ConstraintsZone> constraintsZones = new ArrayList<>();
        constraintsZones.add(new ConstraintsZone(poses.size()-1.3, poses.size()-1, APPROACH_CONSTRAINTS));

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(poses),
                new ArrayList<RotationTarget>(),
                new ArrayList<PointTowardsZone>(),
                constraintsZones,
                new ArrayList<EventMarker>(),
                FAST_CONSTRAINTS,
                new IdealStartingState(chassisSpeedsToVelocity(startSpeeds), currentPose.getRotation()),
                new GoalEndState(0, endPose.getRotation().rotateBy(Rotation2d.k180deg)),
                false
        );
            
        if (AutoBuilder.shouldFlip()) {
            path = path.flipPath();            
        }
        return path;
    }

    public static PathPlannerPath generateStationPath(Pose2d currentPose, int station, int relativePos) {
        return generateStationPath(currentPose, station, relativePos, new ChassisSpeeds());
    }
}
