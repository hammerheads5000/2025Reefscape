// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(MOTOR_ID, Constants.CAN_FD_BUS);
  private PowerDistribution pdh;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(PowerDistribution pdh) {
    this.pdh = pdh;

    SmartDashboard.putData("Climb", climbCommand());
    SmartDashboard.putData("Unclimb", reverseCommand());
    SmartDashboard.putData("Release Electromagnet", releaseIntakeCommand());
    SmartDashboard.putData("Latch Electromagnet", latchIntakeCommand());
  }

  public void stop() {
    motor.setControl(new NeutralOut());
  }

  public void releaseIntake() {
    pdh.setSwitchableChannel(false);
  }

  public void latchIntake() {
    pdh.setSwitchableChannel(true);
  }

  public Command releaseIntakeCommand() {
    return this.runOnce(this::releaseIntake);
  }

  public Command latchIntakeCommand() {
    return this.runOnce(this::latchIntake);
  }

  public Command climbCommand() {
    return this.startEnd(() -> motor.setVoltage(CLIMB_SPEED.in(Volts)), this::stop);
  }

  public Command reverseCommand() {
    return this.startEnd(() -> motor.setVoltage(RELEASE_SPEED.in(Volts)), this::stop);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
