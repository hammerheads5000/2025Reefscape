// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase {

  TalonFX motorLeft;
  TalonFX motorRight;

  DigitalInput frontLidar;
  DigitalInput backLidar;

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    motorLeft = new TalonFX(Constants.EndEffectorConstants.MOTOR_LEFT_ID);
    motorRight = new TalonFX(Constants.EndEffectorConstants.MOTOR_RIGHT_ID);

    frontLidar = new DigitalInput(Constants.EndEffectorConstants.FRONT_LIDAR_ID);
    backLidar = new DigitalInput(Constants.EndEffectorConstants.BACK_LIDAR_ID);
  }

  public void forward() {
    motorLeft.set(Constants.EndEffectorConstants.SPEED);
    motorRight.set(Constants.EndEffectorConstants.SPEED);
  }

  public void stop() {
    motorLeft.setControl(new NeutralOut());
    motorRight.setControl(new NeutralOut());
  }

  public Command forwardCommand() {
    return this.runEnd(() -> forward(), () -> stop());
  }

  public Command intakeCommand() {
    return this.forwardCommand().until(() -> !backLidar.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
