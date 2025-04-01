// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.LightsConstants.ALIGNMENT_PATTERN;
import static frc.robot.Constants.LightsConstants.IDLE_PATTERN;
import static frc.robot.Constants.LightsConstants.PATH_FOLLOWING_PATTERN;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * Pathfind and align to reef
 */
public class ApproachReefCommand extends SequentialCommandGroup {
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

    /** Creates a new ApproachReefCommand. */
    public ApproachReefCommand(int side, double relativePos, Swerve swerve, LightsSubsystem lightsSubsystem) {
        AlignToPoseCommand alignToReefCommand = AlignToReefCommands.alignToReef(side, relativePos, swerve);
        
        Pose2d target = AlignToReefCommands.getReefPose(side, relativePos);

        // don't generate too short of paths
        if (AlignToPoseCommand.withinApproachRange(swerve.getPose(), target)) {
            addCommands(
                    lightsSubsystem.setPatternCommand(ALIGNMENT_PATTERN),
                    alignToReefCommand,
                    lightsSubsystem.setPatternCommand(IDLE_PATTERN)
            );
            return;
        }

        Command followPathCommand = AutoBuilder.followPath(Pathfinding.generateReefPath(swerve.getPose(), side, relativePos, swerve.getFieldSpeeds()));
        
        // First follow generated path
        // Then, alignToReefCommand to ensure alignment
        addCommands(
            lightsSubsystem.setPatternCommand(PATH_FOLLOWING_PATTERN),
            followPathCommand,
            lightsSubsystem.setPatternCommand(ALIGNMENT_PATTERN),
            alignToReefCommand,
            lightsSubsystem.setPatternCommand(IDLE_PATTERN)
        );
    }

    public static Command waitUntilAligningCommand(int side, int relativePos, Swerve swerve) {
        return Commands.waitUntil(() -> AlignToPoseCommand.withinApproachRange(
            swerve.getPose(), AlignToReefCommands.getReefPose(side, relativePos)));
    }

    public static Command waitToDeployElevator(int side, double relativePos, Swerve swerve) {
        return Commands.waitUntil(() -> AlignToPoseCommand.withinRange(
            swerve.getPose(), AlignToReefCommands.getReefPose(side, relativePos), ELEVATOR_DEPLOY_DISTANCE));
    }
}
