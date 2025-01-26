// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
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
    controller = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    elevatorFeedforward = new ElevatorFeedforward(0, 0, 0);
    
    controller.setTolerance(0);

    followerControl.MasterID = motor1.getDeviceID();
    motor2.setControl(followerControl);
  }

  public Distance getHeight() {
    return Meters.of(0);
  }
  
  public void setHeight(Distance height) {
    controller.setGoal(height.in(Meters));
  }

  private void controlUpdate() {
    double output = controller.calculate(getHeight().in(Meters));
    output += elevatorFeedforward.calculate(controller.getSetpoint().velocity);
    motor1.setControl(motorControl.withOutput(output));
  }

  @Override
  public void periodic() {
    controlUpdate();
  }
}
