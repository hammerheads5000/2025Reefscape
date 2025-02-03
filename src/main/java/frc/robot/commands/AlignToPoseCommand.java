// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToPoseCommand extends Command {
    Pose2d targetPose;
    PIDController pidControllerX;
    PIDController pidControllerY;
    PIDController pidControllerAngle;

    private Swerve swerve;

    /** Creates a new AlignToPoseCommand. */
    public AlignToPoseCommand(Pose2d targetPose, ControlConstants<DistanceUnit> pidConstantsX,
            ControlConstants<DistanceUnit> pidConstantsY, ControlConstants<AngleUnit> pidConstantsAngle, Swerve swerve) {
        this.targetPose = targetPose;

        pidControllerX = pidConstantsX.getPIDController();
        pidControllerY = pidConstantsY.getPIDController();
        pidControllerAngle = pidConstantsAngle.getPIDController();

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
        Pose2d relativePose = getPose().relativeTo(targetPose);
        LinearVelocity xVel = MetersPerSecond.of(pidControllerX.calculate(relativePose.getTranslation().getX()));
        LinearVelocity yVel = MetersPerSecond.of(pidControllerY.calculate(relativePose.getTranslation().getY()));
        AngularVelocity angleVel = DegreesPerSecond
                .of(pidControllerAngle.calculate(relativePose.getRotation().getDegrees()));

        swerve.driveRobotCentric(xVel, yVel, angleVel);
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
        return (pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerAngle.atSetpoint());
    }
}
