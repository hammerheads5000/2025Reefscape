// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
  ProfiledPIDController controller;
  ElevatorFeedforward elevatorFeedforward;

  TalonFX motor1;
  TalonFX motor2;

  VoltageOut motorControl;
  Follower followerControl;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    controller = new ProfiledPIDController(kP, kI, kD,
        new TrapezoidProfile.Constraints(MAX_VELOCITY.in(MetersPerSecond),
            MAX_ACCELERATION.in(MetersPerSecondPerSecond)));

    elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);

    controller.setTolerance(TOLERANCE.in(Inches));

    followerControl = new Follower(motor1.getDeviceID(), MOTOR_OPPOSE_DIRECTION);
    motor2.setControl(followerControl);
  }

  public Distance getHeight() {
    return Meters.of(0);
  }

  public void setHeight(Distance height) {
    controller.setGoal(height.in(Meters));
  }

  public void stop() {
    motor1.setControl(new NeutralOut());
  }

  private void controlUpdate() {
    double output = controller.calculate(getHeight().in(Inches));
    output += elevatorFeedforward.calculate(controller.getSetpoint().velocity);
    motor1.setControl(motorControl.withOutput(output));
  }

  @Override
  public void periodic() {
    controlUpdate();
  }

  public Command stopCommand() {
    return this.runOnce(this::stop);
  }

  public Command goToHeightCommand(boolean instant, Distance height) {
    if (instant) {
      return this.runOnce(() -> setHeight(height));
    }
    return this.run(() -> setHeight(height)).until(() -> controller.atGoal());
  }

  public Command goToL1Command(boolean instant) {
    return goToHeightCommand(instant, L1_HEIGHT);
  }

  public Command goToL2Command(boolean instant) {
    return goToHeightCommand(instant, L2_HEIGHT);
  }

  public Command goToL3Command(boolean instant) {
    return goToHeightCommand(instant, L3_HEIGHT);
  }

  public Command goToL4Command(boolean instant) {
    return goToHeightCommand(instant, L4_HEIGHT);
  }

  public Command goToIntakePosCommand(boolean instant) {
    return goToHeightCommand(instant, INTAKE_HEIGHT);
  }

}
