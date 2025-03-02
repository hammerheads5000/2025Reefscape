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
        
        Command followPathCommand = AutoBuilder.followPath(Pathfinding.generateReefPath(swerve.getPose(), side, relativePos, MetersPerSecond.of(initialPIDSpeed)));

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
