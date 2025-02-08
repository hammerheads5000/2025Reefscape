// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
@Logged
public class AlignToPoseCommand extends Command {
    Pose2d targetPose;
    PIDController pidControllerX;
    PIDController pidControllerY;
    PIDController pidControllerAngle;

    private Swerve swerve;

    /** Creates a new AlignToPoseCommand. */
    public AlignToPoseCommand(Pose2d targetPose, ControlConstants pidConstantsX,
            ControlConstants pidConstantsY, ControlConstants pidConstantsAngle, Swerve swerve) {
        this.targetPose = targetPose;

        pidControllerX = pidConstantsX.getPIDController();
        pidControllerY = pidConstantsY.getPIDController();
        pidControllerAngle = pidConstantsAngle.getPIDController();
        pidControllerAngle.enableContinuousInput(-180, 180);

        pidControllerX.setSetpoint(0);
        pidControllerY.setSetpoint(0);
        pidControllerAngle.setSetpoint(0);

        this.swerve = swerve;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d relativeTranslation = targetPose.getTranslation().minus(getPose().getTranslation());
        Rotation2d relativeRotation = targetPose.getRotation().minus(getPose().getRotation());
        LinearVelocity xVel = MetersPerSecond.of(pidControllerX.calculate(relativeTranslation.getX()));
        LinearVelocity yVel = MetersPerSecond.of(pidControllerY.calculate(relativeTranslation.getY()));
        AngularVelocity angleVel = DegreesPerSecond
                .of(pidControllerAngle.calculate(relativeRotation.getDegrees()));

        swerve.driveFieldCentric(xVel, yVel, angleVel);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    private Pose2d getPose() {
        return swerve.getPose();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerAngle.atSetpoint());
    }
}
