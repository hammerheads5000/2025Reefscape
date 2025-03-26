// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.Constants.CONTROLLER_DEADBAND;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

/* Controlling swerve drive in teleop */
public class TeleopSwerve extends Command {
  LinearVelocity driveSpeed = SwerveConstants.DEFAULT_DRIVE_SPEED;
  AngularVelocity rotSpeed = SwerveConstants.DEFAULT_ROT_SPEED;

  // limit how fast speeds can change
  SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(SwerveConstants.MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));
  SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(SwerveConstants.MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));

  private Swerve swerve;
  private CommandXboxController controller;

  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(Swerve swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Applies deadband and normalizes input to [0,1] (not [deadband,1])
  private double processJoystick(double input) {
    if (Math.abs(input) < CONTROLLER_DEADBAND) return 0;
    input -= Math.signum(input)*CONTROLLER_DEADBAND;
    input *= 1 / (1 - CONTROLLER_DEADBAND);

    return input;
  }

  public void setFastSpeed() {
    driveSpeed = SwerveConstants.FAST_DRIVE_SPEED;
    rotSpeed = SwerveConstants.FAST_ROT_SPEED;
  }

  public void setSlowSpeed() {
    driveSpeed = SwerveConstants.SLOW_DRIVE_SPEED;
    rotSpeed = SwerveConstants.SLOW_ROT_SPEED;
  }

  public void setDefaultSpeed() {
    driveSpeed = SwerveConstants.DEFAULT_DRIVE_SPEED;
    rotSpeed = SwerveConstants.DEFAULT_ROT_SPEED;
  }

  public Command fastSpeedCommand() {
    return Commands.runEnd(this::setFastSpeed, this::setDefaultSpeed);
  }

  public Command slowSpeedCommand() {
    return Commands.runEnd(this::setSlowSpeed, this::setDefaultSpeed);
  }

  @Override
  public void execute() {
    // joystick
    double speedX = processJoystick(-controller.getLeftY());
    double speedY = processJoystick(-controller.getLeftX());

    // speedX *= isRed ? -1 : 1;
    // speedY *= isRed ? -1 : 1;

    // raw speed
    speedX = driveSpeed.times(speedX).in(MetersPerSecond);
    speedY = driveSpeed.times(speedY).in(MetersPerSecond);

    // accel limited by slew rate limiter
    speedX = slewRateLimiterX.calculate(speedX);
    speedY = slewRateLimiterY.calculate(speedY);

    swerve.driveFieldCentric(MetersPerSecond.of(speedX),
        MetersPerSecond.of(speedY),
        rotSpeed.times(processJoystick(-controller.getRightX())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
