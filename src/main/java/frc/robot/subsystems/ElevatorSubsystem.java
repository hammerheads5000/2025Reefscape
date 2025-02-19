// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.INST;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.EnumSet;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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

    VoltageOut motorControl;

    boolean enabled = true;

    Angle initMotorPos;

    // To be implemented: LaserCan
    // LaserCan laserCan = new LaserCan(0);

    // MockLaserCan laserCanSim = new MockLaserCan();

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
        controller = CONTROL_CONSTANTS.getProfiledPIDController();

        elevatorFeedforward = CONTROL_CONSTANTS.getElevatorFeedforward();

        motor1 = new TalonFX(MOTOR_1_ID, Constants.CAN_FD_BUS);

        motorControl = new VoltageOut(0);

        motor1.setPosition(0);
        initMotorPos = motor1.getPosition().getValue();

        SmartDashboard.putData("Elevator Sim", mech2d);        
    }

    public void enable() {
        enabled = true;
    }
    
    public void disable() {
        enabled = false;
    }

    public double getMotorRotations() {
        return motor1.getPosition().getValue().in(Rotations);
    }

    public void setBrake(boolean shouldBrake) {
        motor1.setNeutralMode(shouldBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public Distance getHeight() {
        return motorRotationsToHeight(Rotations.of(getMotorRotations()));
    }

    public void setHeight(Distance height) {
        setRotations(heightToMotorRotations(height));
    }

    public void setRotations(double rotations) {
        controller.setGoal(rotations);
    }

    public void setRotations(Angle rotations) {
        controller.setGoal(rotations.in(Rotations));
    }

    public static Distance laserCANtoHeight(Distance measured) {
        Distance yIntercept = MAX_HEIGHT.minus(MIN_LASERCAN_DISTANCE.times(HEIGHT_CHANGE_PER_LASERCAN_DISTANCE));
        return measured.times(HEIGHT_CHANGE_PER_LASERCAN_DISTANCE).plus(yIntercept);
    }

    public static Distance motorRotationsToHeight(Angle rotations) {
        return Meters.of(HEIGHT_PER_MOTOR_ROTATIONS.timesDivisor(rotations).in(Meters));
    }
    
    public static Angle heightToMotorRotations(Distance height) {
        return Rotations.of(height.divideRatio(HEIGHT_PER_MOTOR_ROTATIONS).in(Rotations));
    }

    public double getOutputVolts() {
        return motor1.getMotorVoltage().getValueAsDouble();
    }

    public Distance getSetpoint() {
        return motorRotationsToHeight(Rotations.of(controller.getGoal().position));
    }

    public void stop() {
        motor1.setVoltage(elevatorFeedforward.getKg());
    }

    private void controlUpdate() {
        double output = controller.calculate(getMotorRotations());
        output += elevatorFeedforward.calculate(controller.getSetpoint().velocity);
        motor1.setControl(motorControl.withOutput(output));
    }

    @Override
    public void periodic() {
        if (enabled)
            controlUpdate();
        //System.out.println(setpointEntry.get());
        ligament2d.setLength(elevatorSim.getPositionMeters());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState motor1sim = motor1.getSimState();

        motor1sim.setSupplyVoltage(RobotController.getBatteryVoltage());

        elevatorSim.setInput(motor1sim.getMotorVoltage());
        elevatorSim.update(Constants.SIM_LOOP_PERIOD.in(Seconds));
        
        double rotations = heightToMotorRotations(Meters.of(elevatorSim.getPositionMeters())).in(Rotations);
        double angularVel = heightToMotorRotations(Meters.of(elevatorSim.getVelocityMetersPerSecond())).in(Rotations);
        SmartDashboard.putNumber("idk", elevatorSim.getPositionMeters());

        motor1sim.setRawRotorPosition(rotations);
        motor1sim.setRotorVelocity(angularVel);
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
        return this.startEnd(() -> setHeight(height), () -> {}).until(() -> controller.atGoal());
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
                    Volts.of(1).per(Second),
                    Volts.of(5), null,
                    state -> SignalLogger.writeString("SysId_Elevator_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> 
                motor1.setControl(motorControl.withOutput(output.in(Volts))),
                    null, 
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
