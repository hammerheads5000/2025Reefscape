// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
    TalonFX motor1;
    
    MotionMagicExpoVoltage motorControl;

    boolean enabled = true;

    DCMotor elevatorGearbox = DCMotor.getKrakenX60(1);
    ElevatorSim elevatorSim = new ElevatorSim(elevatorGearbox,
        GEAR_RATIO, CARRIAGE_MASS.in(Kilograms), DRUM_RADIUS.in(Meters), 
        MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters), true, MIN_HEIGHT.in(Meters));

    Mechanism2d mech2d = new Mechanism2d(CANVAS_WIDTH.in(Meters), CANVAS_HEIGHT.in(Meters));
    MechanismRoot2d mechRoot2d = mech2d.getRoot("Elevator Root", ROOT.getX(), ROOT.getY());
    MechanismLigament2d ligament2d = mechRoot2d.append(
        new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90)
    );

    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem() {
        motor1 = new TalonFX(MOTOR_1_ID, Constants.CAN_FD_BUS);
        motor1.getConfigurator().apply(MOTOR_CONFIGS);

        motorControl = new MotionMagicExpoVoltage(0).withEnableFOC(false);
        
        SmartDashboard.putData("Elevator Sim", mech2d);

        SmartDashboard.putData("Reset Elevator Position", resetPositionCommand());
        SmartDashboard.putData("L1", goToL1Command(true));
        SmartDashboard.putData("L2", goToL2Command(true));
        SmartDashboard.putData("L3", goToL3Command(true));
        SmartDashboard.putData("L4", goToL4Command(true));
        SmartDashboard.putData("Intake", goToIntakePosCommand(true));
        SmartDashboard.putData("Intake Jitter", intakeJitterCommand());
        SmartDashboard.putData("Zero", zeroCommand());
    }

    public void resetPosition() {
        motor1.setPosition(0);
    }

    public Angle getPosition() {
        return motor1.getPosition().getValue();
    }

    public void setBrake(boolean shouldBrake) {
        motor1.setNeutralMode(shouldBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    // public Distance getLaserCan() {
    //     return Millimeters.of(laserCan.getMeasurement().distance_mm);
    // }


    public void setRotations(Angle rotations) {
        motor1.setControl(motorControl.withPosition(rotations));
    }

    public void setRotations(double rotations) {
        setRotations(Rotations.of(rotations));
    }

    public void resetAtPosition() {
        setRotations(getPosition());
    }

    public double getOutputVolts() {
        return motor1.getMotorVoltage().getValueAsDouble();
    }

    public Angle getSetpoint() {
        return motorControl.getPositionMeasure();
    }

    public boolean atSetpoint() {
        return motor1.getPosition().getValue().isNear(getSetpoint(), TOLERANCE);
    }

    public Angle getProfileSetpoint() {
        return Rotations.of(motor1.getClosedLoopReference().getValueAsDouble());
    }

    public Voltage getIntegratedOutput() {
        return Volts.of(motor1.getClosedLoopIntegratedOutput().getValueAsDouble());
    }

    public void stop() {
        motor1.setVoltage(GAINS.kG);
    }

    @Override
    public void periodic() {
        ligament2d.setLength(elevatorSim.getPositionMeters());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motor1sim = motor1.getSimState();

        motor1sim.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInput(motor1sim.getMotorVoltage());
        elevatorSim.update(Constants.SIM_LOOP_PERIOD.in(Seconds));
        
        // double rotations = heightToMotorRotations(Meters.of(elevatorSim.getPositionMeters())).in(Rotations);
        // double angularVel = heightToMotorRotations(Meters.of(elevatorSim.getVelocityMetersPerSecond())).in(Rotations);

        // motor1sim.setRawRotorPosition(rotations);
        // motor1sim.setRotorVelocity(angularVel);
    }

    public Command resetPositionCommand() {
        return this.runOnce(this::resetPosition).ignoringDisable(true);
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

    public Command zeroCommand() {
        return this.run(
            () -> motor1.set(MANUAL_DOWN_SPEED))
            .until(() -> motor1.getTorqueCurrent().getValue().abs(Amps) > STALL_CURRENT.in(Amps))
            .andThen(this.runOnce(motor1::disable), // neutral out
                Commands.waitSeconds(1), 
                resetPositionCommand(),
                Commands.waitSeconds(0.5),
                resetPositionCommand(),
                this.runOnce(this::resetAtPosition));
    }

    public Command goToHeightCommand(boolean instant, Angle height) {
        if (instant) {
            return this.runOnce(() -> setRotations(height));
        }
        return this.startEnd(() -> setRotations(height), () -> {}).until(this::atSetpoint);
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

    private Command goToIntakeJitterPosCommand(boolean instant) {
        return goToHeightCommand(instant, INTAKE_HEIGHT.plus(INTAKE_JITTER_AMOUNT));
    }

    public Command intakeJitterCommand() {
        return Commands.repeatingSequence(
            goToIntakePosCommand(true),
            Commands.waitTime(INTAKE_JITTER_PERIOD),
            goToIntakeJitterPosCommand(true),
            Commands.waitTime(INTAKE_JITTER_PERIOD)
        );
    }

    public double getRotations() {
        return getPosition().in(Rotations);
    }

    public double getVelocity() {
        return motor1.getVelocity().getValue().in(RotationsPerSecond);
    }

    public double getVolts() {
        return motor1.getMotorVoltage().getValueAsDouble();
    }

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.75).per(Second),
                    Volts.of(3), null,
                    state -> SmartDashboard.putString("SysId_Elevator_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> 
                motor1.setVoltage(output.in(Volts)),
                    null, 
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
