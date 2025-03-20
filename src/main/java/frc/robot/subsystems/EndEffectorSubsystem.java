// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.EndEffectorConstants.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class EndEffectorSubsystem extends SubsystemBase {

  TalonFX motorLeft;
  TalonFX motorRight;

  DigitalInput frontLidar;
  DigitalInput backLidar;
  DigitalInput intakeLidar;

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    motorLeft = new TalonFX(MOTOR_LEFT_ID, Constants.CAN_FD_BUS);
    motorRight = new TalonFX(MOTOR_RIGHT_ID, Constants.CAN_FD_BUS);

    motorLeft.getConfigurator().apply(MOTOR_LEFT_CONFIGS);
    motorRight.getConfigurator().apply(MOTOR_RIGHT_CONFIGS);

    frontLidar = new DigitalInput(FRONT_LIDAR_ID);
    backLidar = new DigitalInput(BACK_LIDAR_ID);
    intakeLidar = new DigitalInput(INTAKE_LIDAR_ID);

    SmartDashboard.putData("End Effector Intake", intakeCommand());
    SmartDashboard.putData("End Effector Shoot", scoreCommand());
    SmartDashboard.putData("End Effector Trough Left", troughLeftCommand());
    SmartDashboard.putData("End Effector Trough Right", troughRightCommand());
    SmartDashboard.putData("End Effector Reverse", forwardCommand(-INTAKE_SPEED));
  }

  public boolean getFrontLidar() {
    return frontLidar.get();
  }

  public boolean getBackLidar() {
    return backLidar.get();
  }
  
  public boolean getIntakeLidar() {
    return intakeLidar.get();
  }

  public void forward(double speedDutyCycle) {
    motorLeft.set(speedDutyCycle);
    motorRight.set(speedDutyCycle);
  }

  public void setMotors(double left, double right) {
    motorLeft.set(left);
    motorRight.set(right);
  }

  public void stop() {
    motorLeft.setControl(new NeutralOut());
    motorRight.setControl(new NeutralOut());
  }

  public Command forwardCommand(double speedDutyCycle) {
    return this.startEnd(() -> forward(speedDutyCycle), () -> stop());
  }

  public Command intakeCommand() {
    return this.forwardCommand(INTAKE_SPEED).until(() -> !backLidar.get())
        .andThen(this.forwardCommand(SLOW_INTAKE_SPEED).until(() -> backLidar.get() && !frontLidar.get()));
  }

  public Command scoreCommand() {
    return this.forwardCommand(SCORE_SPEED).until(() -> frontLidar.get());
  }

  public Command troughLeftCommand() {
    return this.startEnd(() -> setMotors(SLOW_TROUGH_SPEED, FAST_TROUGH_SPEED), this::stop)
        .until(() -> frontLidar.get());
  }

  public Command troughRightCommand() {
    return this.startEnd(() -> setMotors(FAST_TROUGH_SPEED, SLOW_TROUGH_SPEED), this::stop)
        .until(() -> frontLidar.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
