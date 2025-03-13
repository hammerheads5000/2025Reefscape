// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.AutoConstants.APPROACH_DISTANCE;
import static frc.robot.Constants.AutoConstants.PULL_DISTANCE;
import static frc.robot.Constants.ElevatorConstants.L3_HEIGHT;
import static frc.robot.Constants.SwerveConstants.SCORING_PID_ANGLE;
import static frc.robot.Constants.SwerveConstants.SCORING_PID_X;
import static frc.robot.Constants.SwerveConstants.SCORING_PID_Y;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RemoveAlgaeCommand extends SequentialCommandGroup {
    private Pose2d getAlgaePose(int side) {
        Pose2d reefPose = AlignToReefCommands.getReefPose(side, 0);
        return reefPose.rotateAround(reefPose.getTranslation(), Rotation2d.kCCW_90deg);
    }

    /** Creates a new RemoveAlgaeCommand. */
    public RemoveAlgaeCommand(Swerve swerve, ElevatorSubsystem elevatorSubsystem) {
        Pose2d pose = swerve.getPose();
        int side = Pathfinding.getClosestReefSide(pose);
        
        Pose2d algaePose = getAlgaePose(side);
        Translation2d pullTranslation = new Translation2d(Meters.zero(), PULL_DISTANCE);
        Pose2d pullPose = algaePose.transformBy(new Transform2d(pullTranslation, Rotation2d.kZero));


        addCommands(
            Commands.parallel(
                new AlignToPoseCommand(algaePose, SCORING_PID_X, SCORING_PID_Y, SCORING_PID_ANGLE, swerve),
                elevatorSubsystem.goToL4Command(false)
            ),
            elevatorSubsystem.goToIntakePosCommand(true),
            Commands.waitUntil(() -> elevatorSubsystem.getPosition().lt(L3_HEIGHT)),
            new AlignToPoseCommand(pullPose, SCORING_PID_ANGLE, SCORING_PID_X, SCORING_PID_Y, swerve)
        );
    }
}
