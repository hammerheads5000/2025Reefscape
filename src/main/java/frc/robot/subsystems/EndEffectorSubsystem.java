// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

  TalonFX motorLeft;
  TalonFX motorRight;

  DigitalInput frontLidar;
  DigitalInput backLidar;

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    motorLeft = new TalonFX(MOTOR_LEFT_ID);
    motorRight = new TalonFX(MOTOR_RIGHT_ID);

    motorLeft.getConfigurator().apply(MOTOR_LEFT_CONFIGS);

    frontLidar = new DigitalInput(FRONT_LIDAR_ID);
    backLidar = new DigitalInput(BACK_LIDAR_ID);
  }

  public void forward(double speedDutyCycle) {
    motorLeft.set(speedDutyCycle);
    motorRight.set(speedDutyCycle);
  }

  public void stop() {
    motorLeft.setControl(new NeutralOut());
    motorRight.setControl(new NeutralOut());
  }

  public Command forwardCommand(double speedDutyCycle) {
    return this.runEnd(() -> forward(speedDutyCycle), () -> stop());
  }

  public Command intakeCommand() {
    return this.forwardCommand(INTAKE_SPEED).until(() -> !backLidar.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
