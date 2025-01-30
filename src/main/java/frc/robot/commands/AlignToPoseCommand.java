// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToPoseCommand extends Command {
    Pose2d targetPose;

    /** Creates a new AlignToPoseCommand. */
    public AlignToPoseCommand(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    private Pose2d getPose() {
        return new Pose2d();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (
            getPose().getTranslation().getMeasureX().minus(targetPose.getTranslation().getMeasureX()).lte(SCORING_ALIGN_TOLERANCE_X)
            && getPose().getTranslation().getMeasureY().minus(targetPose.getTranslation().getMeasureY()).lte(SCORING_ALIGN_TOLERANCE_Y)
            && getPose().getRotation().getMeasure().minus(targetPose.getRotation().getMeasure()).lte(SCORING_ANGLE_TOLERANCE)
        );
    }
}
