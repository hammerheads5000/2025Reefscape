// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ControlConstants {
    // PID gains
    double kP, kI, kD;
    double tolerance;
    double iZone = Double.POSITIVE_INFINITY;
    double iMin = Double.NEGATIVE_INFINITY;
    double iMax = Double.POSITIVE_INFINITY;

    // feedforward gains
    double kV, kA;

    // physical gains
    double kS, kG;

    // trapezoid profile
    double maxVel;
    double maxAcc;

    public ControlConstants() {
    }

    public ControlConstants withPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        return this;
    }

    public ControlConstants withFeedforward(double kV, double kA) {
        this.kV = kV;
        this.kA = kA;
        return this;
    }
    
    public ControlConstants withPhysical(double kS, double kG) {
        this.kS = kS;
        this.kG = kG;
        return this;
    }

    public ControlConstants withProfile(double maxVel, double maxAcc) {
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        return this;
    }


    public ControlConstants withTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public ControlConstants withIZone(double iZone) {
        this.iZone = iZone;
        return this;
    }
    
    public ControlConstants withIRange(double iMin, double iMax) {
        this.iMin = iMin;
        this.iMax = iMax;
        return this;
    }

    public PIDController getPIDController() {
        PIDController controller = new PIDController(kP, kI, kD);
        controller.setTolerance(tolerance);
        controller.setIntegratorRange(iMin, iMax);
        controller.setIZone(iZone);

        return controller;
    }

    public ProfiledPIDController getProfiledPIDController() {
        ProfiledPIDController controller = new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(maxVel, maxAcc)
        );
        controller.setTolerance(tolerance);
        controller.setIntegratorRange(iMin, iMax);
        controller.setIZone(iZone);

        return controller;
    }

    public ElevatorFeedforward getElevatorFeedforward() {
        return new ElevatorFeedforward(kS, kG, kV, kA);
    }
}