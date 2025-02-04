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
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

    TorqueCurrentFOC motorControl;
    Follower followerControl;

    boolean enabled = true;

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
        controller = CONTROL_CONSTANTS.getProfiledPIDController();

        elevatorFeedforward = CONTROL_CONSTANTS.getElevatorFeedforward();

        followerControl = new Follower(motor1.getDeviceID(), MOTOR_OPPOSE_DIRECTION);
        motor2.setControl(followerControl);
    }

    public void enable() {
        enabled = true;
    }
    
    public void disable() {
        enabled = false;
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
        if (enabled)
            controlUpdate();
    }

    public Command stopCommand() {
        return this.runOnce(this::stop);
    }

    public Command moveUpManualCommand() {
        return this.runEnd(() -> motor1.setControl(motorControl.withOutput(MANUAL_UP_SPEED)), this::stop);
    }

    public Command moveDownManualCommand() {
        return this.runEnd(() -> motor1.setControl(motorControl.withOutput(MANUAL_DOWN_SPEED)), this::stop);
    }

    public Command goToHeightCommand(boolean instant, Distance height) {
        if (instant) {
            return this.runOnce(() -> setHeight(height));
        }
        return this.startEnd(() -> setHeight(height), null).until(() -> controller.atGoal());
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
