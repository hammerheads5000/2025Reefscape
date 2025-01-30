// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static class SwerveConstants {
        // Aligning to the reef to score coral

        // output: m/s, measure: m
        public static final ControlConstants<DistanceUnit> SCORING_PID_X = new ControlConstants<DistanceUnit>(
                1.0, 0.0, 0.0, Inches.of(2));
        public static final ControlConstants<DistanceUnit> SCORING_PID_Y = new ControlConstants<DistanceUnit>(
                1.0, 0.0, 0.0, Inches.of(4));
        
        // output: deg/s, measure: deg
        public static final ControlConstants<AngleUnit> SCORING_PID_ANGLE = new ControlConstants<AngleUnit>(
                1.0, 0.0, 0.0, Degrees.of(5));

    }

    public static class ElevatorConstants {
        // Motors
        public static final int MOTOR_1_ID = 12;
        public static final int MOTOR_2_ID = 13;
        public static final boolean MOTOR_OPPOSE_DIRECTION = true; // whether motor 2 rotates the same direction as
                                                                   // motor 1
        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(20));

        // Control (amps, inches)
        public static final ControlConstants<DistanceUnit> CONTROL_CONSTANTS = new ControlConstants<DistanceUnit>(
            1.0, 0.0, 0.0, Inches.of(1),
            0, 0, 0, 0,
            InchesPerSecond.ofNative(72), InchesPerSecond.ofNative(72).per(Second)
        );

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

        // Speed (duty cycle)
        public static final double INTAKE_SPEED = 1;
        public static final double SCORE_SPEED = 1;
        public static final double FAST_TROUGH_SPEED = 0.5;
        public static final double SLOW_TROUGH_SPEED = 0.25;
    }

    public static class FieldConstants {
        public static final AprilTagFieldLayout APRIL_TAGS = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        public static final Pose2d LEFT_CORAL_STATION = new Pose2d(
                Meters.of(1.16), Meters.of(7.013),
                Rotation2d.fromDegrees(125));

        public static final Pose2d RIGHT_CORAL_STATION = new Pose2d(
                Meters.of(1.16), Meters.of(1.02),
                Rotation2d.fromDegrees(-125));

        public static final Distance DISTANCE_TO_REEF = Inches.of(29 / 2);
    }
}
