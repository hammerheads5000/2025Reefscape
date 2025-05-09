// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.SWEEP_OFFSET;
import static frc.robot.Constants.FieldConstants.SWEEP_RELATIVE_POS;
import static frc.robot.Constants.SwerveConstants.SWEEP_PID_ANGLE;
import static frc.robot.Constants.SwerveConstants.SWEEP_PID_TRANSLATION;
import static frc.robot.Constants.SwerveConstants.SWEEP_PID_TRANSLATION;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SweepCommand extends SequentialCommandGroup {
    /** Creates a new SweepCommand. */
    public SweepCommand(Swerve swerve) {
        Pose2d pose = swerve.getPose();
        int side = Pathfinding.getClosestReefSide(swerve.getPose());
        Pose2d leftPose = AlignToReefCommands.getReefPose(side, SWEEP_RELATIVE_POS);
        Pose2d rightPose = AlignToReefCommands.getReefPose(side, -SWEEP_RELATIVE_POS);
        Pose2d centerPose = AlignToReefCommands.getReefPose(side, 0);
        Pose2d pose1;
        if (leftPose.getTranslation().getDistance(pose.getTranslation())
            < rightPose.getTranslation().getDistance(pose.getTranslation())) {
            pose1 = leftPose;
        } else {
            pose1 = rightPose;
        }
        pose1 = pose1.transformBy(new Transform2d(new Translation2d(SWEEP_OFFSET.unaryMinus(), Meters.zero()), Rotation2d.kZero));
        addCommands(
            new AlignToPoseCommand(pose1, SWEEP_PID_TRANSLATION, SWEEP_PID_ANGLE, swerve),
            new AlignToPoseCommand(centerPose, SWEEP_PID_TRANSLATION, SWEEP_PID_ANGLE, swerve)            
        );
    }
}
