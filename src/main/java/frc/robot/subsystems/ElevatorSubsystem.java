// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.simulation.MockLaserCan;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
    ProfiledPIDController controller;
    ElevatorFeedforward elevatorFeedforward;

    TalonFX motor1;
    TalonFX motor2;

    TorqueCurrentFOC motorControl;
    Follower followerControl;

    boolean enabled = true;

    Angle initMotorPos;

    // To be implemented: LaserCan
    // LaserCan laserCan = new LaserCan(0);

    // MockLaserCan laserCanSim = new MockLaserCan();

    DCMotor elevatorGearbox = DCMotor.getKrakenX60(2);
    ElevatorSim elevatorSim = new ElevatorSim(elevatorGearbox,
        GEAR_RATIO, CARRIAGE_MASS.in(Kilograms), DRUM_RADIUS.in(Meters), 
        MAX_HEIGHT.in(Meters), MIN_HEIGHT.in(Meters), true, MIN_HEIGHT.in(Meters));

    Mechanism2d mech2d = new Mechanism2d(CANVAS_WIDTH.in(Meters), CANVAS_HEIGHT.in(Meters));
    MechanismRoot2d mechRoot2d = mech2d.getRoot("Elevator Root", ROOT.getX(), ROOT.getY());
    MechanismLigament2d ligament2d = mechRoot2d.append(
        new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90)
    );

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
        controller = CONTROL_CONSTANTS.getProfiledPIDController();

        elevatorFeedforward = CONTROL_CONSTANTS.getElevatorFeedforward();

        motor1 = new TalonFX(MOTOR_1_ID, Constants.CAN_FD_BUS);
        motor2 = new TalonFX(MOTOR_2_ID, Constants.CAN_FD_BUS);

        motorControl = new TorqueCurrentFOC(0).withDeadband(Amps.of(5));
        followerControl = new Follower(motor1.getDeviceID(), MOTOR_OPPOSE_DIRECTION);
        motor2.setControl(followerControl);

        initMotorPos = motor1.getPosition().getValue();

        SmartDashboard.putData("Eevator Sim", mech2d);
    }

    public void enable() {
        enabled = true;
    }
    
    public void disable() {
        enabled = false;
    }

    public double getMotorRotations() {
        return motor1.getPosition().getValue().minus(initMotorPos).in(Rotations);
    }

    public void setBrake(boolean shouldBrake) {
        motor1.getConfigurator().apply(shouldBrake ? BRAKE_CONFIGS : COAST_CONFIGS);
    }

    public Distance getHeight() {
        return Meters.of(elevatorSim.getPositionMeters());
    }

    public void setHeight(Distance height) {
        controller.setGoal(height.in(Meters));
    }

    public Rotation2d heightToMotorRotations(Distance height) {
        return Rotation2d.kZero;
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

        ligament2d.setLength(elevatorSim.getPositionMeters());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motor1sim = motor1.getSimState();
        TalonFXSimState motor2sim = motor2.getSimState();

        motor1sim.setSupplyVoltage(RobotController.getBatteryVoltage());
        motor2sim.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInput(motor1sim.getMotorVoltage() + motor2sim.getMotorVoltage());
        elevatorSim.update(Constants.SIM_LOOP_PERIOD.in(Seconds));

        double rotations = heightToMotorRotations(Meters.of(elevatorSim.getPositionMeters())).getRotations();
        double angularVel = heightToMotorRotations(Meters.of(elevatorSim.getVelocityMetersPerSecond())).getRotations();

        motor1sim.setRawRotorPosition(rotations);
        motor1sim.setRotorVelocity(angularVel);

        motor2sim.setRawRotorPosition(rotations);
        motor2sim.setRotorVelocity(angularVel);
    }

    public Command stopCommand() {
        return this.runOnce(this::stop);
    }

    public Command moveUpManualCommand() {
        return this.runEnd(() -> motor1.set(MANUAL_UP_SPEED), this::stop);
    }

    public Command moveDownManualCommand() {
        return this.runEnd(() -> motor1.set(MANUAL_DOWN_SPEED), this::stop);
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

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(2).per(Second),
                    Volts.of(15), null,
                    state -> SignalLogger.writeString("SysId_Elevator_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> {
                motor1.setControl(motorControl.withOutput(output.in(Volts)));
                System.out.println(output.in(Volts));},
                    null, 
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
