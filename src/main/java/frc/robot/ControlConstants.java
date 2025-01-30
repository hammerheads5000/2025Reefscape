// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;

public class ControlConstants<U extends Unit> {
    // PID gains
    double kP, kI, kD;
    Measure<U> tolerance;

    // feedforward gains
    double kV, kA;

    // physical gains
    double kS, kG;

    // trapezoid profile
    Per<U, TimeUnit> maxVel;
    Velocity<PerUnit<U, TimeUnit>> maxAcc;

    public ControlConstants(double kP, double kI, double kD, Measure<U> tolerance) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tolerance = tolerance;
    }

    public ControlConstants(double kV, double kA, double kS, double kG) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.kG = kG;
    }

    public ControlConstants(double kV, double kA) {
        this(kV, kA, 0, 0);
    }

    public ControlConstants(double kP, double kI, double kD, Measure<U> tolerance,
        Per<U, TimeUnit> maxVel, Velocity<PerUnit<U, TimeUnit>> maxAcc) {
        this(kP, kI, kD, tolerance);
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    public ControlConstants(double kV, double kA, double kS, double kG,
        Per<U, TimeUnit> maxVel, Velocity<PerUnit<U, TimeUnit>> maxAcc) {
        this(kV, kA, kS, kG);
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    public ControlConstants(double kP, double kI, double kD, Measure<U> tolerance,
            double kV, double kA, double kS, double kG,
            Per<U, TimeUnit> maxVel, Velocity<PerUnit<U, TimeUnit>> maxAcc) {
        this(kP, kI, kD, tolerance, maxVel, maxAcc);
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.kG = kG;
    }

    public PIDController getPIDController() {
        PIDController controller = new PIDController(kP, kI, kD);
        controller.setTolerance(tolerance.baseUnitMagnitude());

        return controller;
    }

    public ProfiledPIDController getProfiledPIDController() {
        ProfiledPIDController controller = new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(maxVel.baseUnitMagnitude(), maxAcc.baseUnitMagnitude())
        );
        controller.setTolerance(tolerance.baseUnitMagnitude());

        return controller;
    }

    public ElevatorFeedforward getElevatorFeedforward() {
        return new ElevatorFeedforward(kS, kG, kV, kA);
    }
}