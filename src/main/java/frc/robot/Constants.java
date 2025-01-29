// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static class ElevatorConstants {
        // Motors
        public static final int MOTOR_1_ID = 12;
        public static final int MOTOR_2_ID = 13;
        public static final boolean MOTOR_OPPOSE_DIRECTION = true; // whether motor 2 rotates the same direction as
                                                                   // motor 1
        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(20));

        // PID
        public static final double kP = 1.0; // amps per inch error
        public static final double kI = 0; // amps per inch-seconds error
        public static final double kD = 0; // amps per inch per second error
        public static final Distance TOLERANCE = Inches.of(1);

        // Feedforward
        public static final double kS = 0; // amps to overcome static friction
        public static final double kG = 0; // amps to overcome gravity
        public static final double kV = 0; // feedforward amps per inch per second
        public static final double kA = 0; // feedforward amps per inch per second^2

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

    public static class EndEffectorConstants {
        // Motors
        public static final int MOTOR_LEFT_ID = 0;
        public static final int MOTOR_RIGHT_ID = 0;
        
        public static final MotorOutputConfigs MOTOR_LEFT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);
        public static final MotorOutputConfigs MOTOR_RIGHT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);
        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(10));

        // Lidar
        public static final int FRONT_LIDAR_ID = 0;
        public static final int BACK_LIDAR_ID = 0;

        // Speed
        public static final double INTAKE_SPEED = 1; // duty cycle out
        public static final double OUTAKE_SPEED = 1;
    }
}
