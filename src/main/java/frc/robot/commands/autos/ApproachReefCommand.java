// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static frc.robot.Constants.AutoConstants.*;

import java.util.ArrayList;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.subsystems.Swerve;

/**
 * Pathfind and align to reef
 */
public class ApproachReefCommand extends SequentialCommandGroup {
    /**
     * @param pose Robot pose to compare
     * @return What side of the reef the robot is closest to (0 -> 5 CW starting at far left)
     */
    private static int getClosestReefSide(Pose2d pose) {
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

    /**
     * @return Side one step closer to targetSide from currentSide
     */
    private static int getNextSide(int currentSide, int targetSide) {
        int positiveDistance = distanceBetweenSides(currentSide + 1, targetSide);
        int negativeDistance = distanceBetweenSides(currentSide - 1, targetSide);

        if (positiveDistance > negativeDistance) return (currentSide + 1 + 6) % 6;
        return (currentSide - 1 + 6) % 6;
    }

    /**
     * Generate PathPlannerPath to the desired branch, moving around the reef if necessary
     */
    private static PathPlannerPath generatePath(Pose2d currentPose, int side, int relativePos, LinearVelocity endVelocity) {
        int currentSide = getClosestReefSide(currentPose);
        
        ArrayList<Pose2d> poses = new ArrayList<>();
        while (distanceBetweenSides(currentSide, side) > 1) {
            int nextSide = getNextSide(currentSide, side);
            Pose2d sidePose = AlignToReefCommands.getReefPose(nextSide, 0);
            
            sidePose.transformBy(new Transform2d(new Translation2d(TRAVERSE_DISTANCE.unaryMinus(), Meters.zero()), Rotation2d.kZero));
            
            if (nextSide == (currentSide + 1 + 6) % 6) {
                sidePose.rotateAround(sidePose.getTranslation(), Rotation2d.kCCW_90deg);
            } else {
                sidePose.rotateAround(sidePose.getTranslation(), Rotation2d.kCW_90deg);
            }

            poses.add(sidePose);
            currentSide = nextSide;
        }
        Pose2d endPose = AlignToReefCommands.getReefPose(side, relativePos);
        poses.add(endPose);

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(poses), 
                CONSTRAINTS,
                null,
                new GoalEndState(endVelocity.in(MetersPerSecond), endPose.getRotation())
        );
        path.preventFlipping = false;
        return path;
    }

    private static void overrideFeedback(Swerve swerve, AlignToPoseCommand alignToPoseCommand) {
        PPHolonomicDriveController.overrideXFeedback(() -> {
            Pose2d pose = swerve.getPose();
            double xVel = alignToPoseCommand.pidControllerX.calculate(pose.getX());

            return xVel;
        });

        PPHolonomicDriveController.overrideYFeedback(() -> {
            Pose2d pose = swerve.getPose();
            double yVel = alignToPoseCommand.pidControllerY.calculate(pose.getY());

            return yVel;
        });

        PPHolonomicDriveController.overrideRotationFeedback(() -> {
            Pose2d pose = swerve.getPose();
            double angleVel = DegreesPerSecond
                .of(alignToPoseCommand.pidControllerAngle.calculate(pose.getRotation().getDegrees())).in(RadiansPerSecond);

            return angleVel;
        });
    }

    private static void resetOverrideFeedback() {
        PPHolonomicDriveController.clearFeedbackOverrides();
    }

    private static boolean withinApproachRange(Pose2d pose, Pose2d target) {
        return pose.getTranslation().getDistance(target.getTranslation()) <= APPROACH_DISTANCE.in(Meters);
    }

    /** Creates a new ApproachReefCommand. */
    public ApproachReefCommand(int side, int relativePos, Swerve swerve) {
        AlignToPoseCommand alignToReefCommand = AlignToReefCommands.alignToReef(side, relativePos, swerve);
        
        Pose2d approachTarget = AlignToReefCommands.getReefPose(side, relativePos);
        
        Translation2d shiftApproachTransform = new Translation2d(APPROACH_DISTANCE.unaryMinus(), Meters.zero());
        approachTarget = approachTarget.plus(new Transform2d(shiftApproachTransform, Rotation2d.kZero));

        double relativeApproachX = approachTarget.minus(alignToReefCommand.targetPose).getX();
        double relativeApproachY = approachTarget.minus(alignToReefCommand.targetPose).getY();
        Vector<N2> initialPIDVelocity = VecBuilder.fill(relativeApproachX, relativeApproachY);
        Translation2d direction = new Translation2d(1, 0).rotateBy(alignToReefCommand.targetPose.getRotation());
        double initialPIDSpeed = initialPIDVelocity.projection(VecBuilder.fill(direction.getX(), direction.getY())).norm();

        //Command pathFindingCommand = AutoBuilder.pathfindToPose(approachTarget, CONSTRAINTS, initialPIDSpeed);
        Command followPathCommand = AutoBuilder.followPath(generatePath(swerve.getPose(), side, relativePos, MetersPerSecond.of(initialPIDSpeed)));

        // First follow generated path
        // When within approach distance while following path, override PP's feedback
        // Finally, alignToReefCommand to ensure alignment
        addCommands(
            followPathCommand.alongWith(
                    Commands.idle().until(() -> withinApproachRange(
                    swerve.getPose(), AlignToReefCommands.getReefPose(side, relativePos)))
                    .andThen(Commands.runOnce(() -> overrideFeedback(swerve, alignToReefCommand)))
            ).andThen(Commands.runOnce(ApproachReefCommand::resetOverrideFeedback)),
            alignToReefCommand
        );
    }
}
