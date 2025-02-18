// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static frc.robot.Constants.AutoConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ApproachReefCommand extends SequentialCommandGroup {
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

        Command pathFindingCommand = AutoBuilder.pathfindToPose(approachTarget, CONSTRAINTS, initialPIDSpeed);
        addCommands(
            pathFindingCommand,
            alignToReefCommand
        );
    }
}
