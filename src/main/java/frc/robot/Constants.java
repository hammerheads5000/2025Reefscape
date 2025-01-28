// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    public class ElevatorConstants {
        public static final Voltage MAX_OUTPUT = Volts.of(12);
        public static final Voltage MIN_OUTPUT = Volts.of(-12);

        // Motors
        public static final int MOTOR_1_ID = 0;
        public static final int MOTOR_2_ID = 0;
        public static final boolean MOTOR_OPPOSE_DIRECTION = false; // whether motor 2 rotates the same direction as motor 1

        // PID
        public static final double kP = 1.0; // volts per inch error
        public static final double kI = 0; // volts per inch-seconds error
        public static final double kD = 0; // volts per inch per second error
        public static final Distance TOLERANCE = Inches.of(1);

        // Feedforward
        public static final double kS = 0; // volts to overcome static friction
        public static final double kG = 0; // volts to overcome gravity
        public static final double kV = 0; // feedforward volts per inch per second
        public static final double kA = 0; // feedforward volts per inch per second^2

        // Trapezoid profile constraints
        public static final LinearVelocity MAX_VELOCITY = InchesPerSecond.of(72);
        public static final LinearAcceleration MAX_ACCELERATION = InchesPerSecond.of(72).div(Seconds.of(1));

        // Setpoints
        public static final Distance L1_HEIGHT = Inches.of(0);
        public static final Distance L2_HEIGHT = Inches.of(0);
        public static final Distance L3_HEIGHT = Inches.of(0);
        public static final Distance L4_HEIGHT = Inches.of(0);
        public static final Distance INTAKE_HEIGHT = Inches.of(0);
    }

}
