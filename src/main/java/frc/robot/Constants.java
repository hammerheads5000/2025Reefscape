// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    public static final CANBus CAN_FD_BUS = new CANBus("Bobby");
    public static final CANBus CAN_RIO_BUS = new CANBus("rio");

    public static final Time SIM_LOOP_PERIOD = Milliseconds.of(20);

    public static final NetworkTableInstance INST = NetworkTableInstance.getDefault();

    public static final double CONTROLLER_DEADBAND = .225;
    

    public static class SwerveConstants {
        public static final LinearVelocity DEFAULT_DRIVE_SPEED = MetersPerSecond.of(3);//define later
        public static final AngularVelocity DEFAULT_ROT_SPEED = RotationsPerSecond.of(1.5);

        public static final LinearVelocity FAST_DRIVE_SPEED = MetersPerSecond.of(8);//find values later
        public static final AngularVelocity FAST_ROT_SPEED = RotationsPerSecond.of(4);

        public static final LinearVelocity SLOW_DRIVE_SPEED = MetersPerSecond.of(1);
        public static final AngularVelocity SLOW_ROT_SPEED = RotationsPerSecond.of(0.5);

        public static final LinearAcceleration MAX_TELEOP_ACCEL = MetersPerSecondPerSecond.of(15);

        // distance between modules on same side (front to back or left to right)
        private static final Distance MODULE_DISTANCE = Inches.of(23.75);

        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
                .withKP(56).withKI(10).withKD(9)
                .withKS(2.38).withKV(0.29).withKA(1.23)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(0.133).withKI(0).withKD(0)
                .withKS(0.182).withKV(0.124);

        private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.TorqueCurrentFOC;
        private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

        private static final Current SLIP_CURRENT = Amps.of(120.0); // NEEDS TUNING

        private static final TalonFXConfiguration DRIVE_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withStatorCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake));

        private static final TalonFXConfiguration STEER_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(60))
                        .withStatorCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake));

        private static final CANcoderConfiguration ENCODER_CONFIGS = new CANcoderConfiguration();

        private static final Pigeon2Configuration PIGEON_CONFIGS = new Pigeon2Configuration().withMountPose(
                new MountPoseConfigs().withMountPoseYaw(Degrees.of(-90)).withMountPosePitch(Degrees.of(180)));

        public static final LinearVelocity SPEED_AT_12V = MetersPerSecond.of(4.73); // NEEDS TUNING

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        private static final double COUPLE_RATIO = 3.5714285714285716;

        private static final double DRIVE_GEAR_RATIO = 6.746031746031747;
        private static final double STEER_GEAR_RATIO = 21.428571428571427;
        private static final Distance WHEEL_RADIUS = Inches.of(2);

        private static final int PIGEON_ID = 1;

        // SIMULATION inertia
        private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
        // SIMULATION voltage necessary to overcome friction
        private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
        private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
                .withCANBusName(CAN_FD_BUS.getName())
                .withPigeon2Id(PIGEON_ID)
                .withPigeon2Configs(PIGEON_CONFIGS);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> CONSTANT_CREATOR = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withWheelRadius(WHEEL_RADIUS)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                .withSlipCurrent(SLIP_CURRENT)
                .withSpeedAt12Volts(SPEED_AT_12V)
                .withDriveMotorType(DRIVE_MOTOR_TYPE)
                .withSteerMotorType(STEER_MOTOR_TYPE)
                .withFeedbackSource(STEER_FEEDBACK_TYPE)
                .withDriveMotorInitialConfigs(DRIVE_CONFIGS)
                .withSteerMotorInitialConfigs(STEER_CONFIGS)
                .withEncoderInitialConfigs(ENCODER_CONFIGS)
                .withSteerInertia(STEER_INERTIA)
                .withDriveInertia(DRIVE_INERTIA)
                .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

        public static class FrontLeft {
            private static final int DRIVE_ID = 3;
            private static final int STEER_ID = 7;
            private static final int ENCODER_ID = 3;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.25830078125);
            private static final boolean STEER_INVERTED = true;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = false;

            private static final Distance X_POS = MODULE_DISTANCE.div(2);
            private static final Distance Y_POS = MODULE_DISTANCE.div(2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS = CONSTANT_CREATOR
                    .createModuleConstants(
                            STEER_ID, DRIVE_ID, ENCODER_ID, ENCODER_OFFSET, X_POS, Y_POS,
                            DRIVE_INVERTED, STEER_INVERTED, ENCODER_INVERTED);
        }

        public static class FrontRight {
            private static final int DRIVE_ID = 4;
            private static final int STEER_ID = 8;
            private static final int ENCODER_ID = 4;
            private static final Angle ENCODER_OFFSET = Rotations.of(0.361083984375);
            private static final boolean STEER_INVERTED = true;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = true;

            private static final Distance X_POS = MODULE_DISTANCE.div(2);
            private static final Distance Y_POS = MODULE_DISTANCE.div(-2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS = CONSTANT_CREATOR
                    .createModuleConstants(
                            STEER_ID, DRIVE_ID, ENCODER_ID, ENCODER_OFFSET, X_POS, Y_POS,
                            DRIVE_INVERTED, STEER_INVERTED, ENCODER_INVERTED);
        }

        public static class BackLeft {
            private static final int DRIVE_ID = 2;
            private static final int STEER_ID = 6;
            private static final int ENCODER_ID = 2;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.0693359375);
            private static final boolean STEER_INVERTED = true;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = false;

            private static final Distance X_POS = MODULE_DISTANCE.div(-2);
            private static final Distance Y_POS = MODULE_DISTANCE.div(2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS = CONSTANT_CREATOR
                    .createModuleConstants(
                            STEER_ID, DRIVE_ID, ENCODER_ID, ENCODER_OFFSET, X_POS, Y_POS,
                            DRIVE_INVERTED, STEER_INVERTED, ENCODER_INVERTED);
        }

        public static class BackRight {
            private static final int DRIVE_ID = 1;
            private static final int STEER_ID = 5;
            private static final int ENCODER_ID = 1;
            private static final Angle ENCODER_OFFSET = Rotations.of(0.268798828125);
            private static final boolean STEER_INVERTED = true;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = true;

            private static final Distance X_POS = MODULE_DISTANCE.div(-2);
            private static final Distance Y_POS = MODULE_DISTANCE.div(-2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS = CONSTANT_CREATOR
                    .createModuleConstants(
                            STEER_ID, DRIVE_ID, ENCODER_ID, ENCODER_OFFSET, X_POS, Y_POS,
                            DRIVE_INVERTED, STEER_INVERTED, ENCODER_INVERTED);
        }

        public static final Frequency ODOMETRY_UPDATE_FREQ = Hertz.of(0); // 0 Hz = default 250 Hz for CAN FD
        public static final Matrix<N3, N1> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.01);
        public static final Matrix<N3, N1> VISION_STD_DEV = VecBuilder.fill(0.1, 0.1, 0.05);

        public static final DriveRequestType DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;
        public static final SteerRequestType STEER_REQUEST_TYPE = SteerRequestType.MotionMagicExpo;
        
        public static final LinearVelocity LINEAR_VEL_DEADBAND = MetersPerSecond.of(0.1);
        public static final AngularVelocity ANGLULAR_VEL_DEADBAND = DegreesPerSecond.of(5);

        // Aligning to the reef to score coral

        // output: m/s, measure: m
        public static final ControlConstants<DistanceUnit> SCORING_PID_X = new ControlConstants<DistanceUnit>(
                1.75, 0.0, 0.0, Inches.of(2));
        public static final ControlConstants<DistanceUnit> SCORING_PID_Y = new ControlConstants<DistanceUnit>(
                1.75, 0.0, 0.0, Inches.of(2));

        // output: deg/s, measure: deg
        public static final ControlConstants<AngleUnit> SCORING_PID_ANGLE = new ControlConstants<AngleUnit>(
                2.5, 0.0, 0.0, Degrees.of(2));

    }

    public static class AutoConstants {
        // Test Autos
        public static final PathConstraints CONSTRAINTS = new PathConstraints(
                SwerveConstants.DEFAULT_DRIVE_SPEED,
                SwerveConstants.DEFAULT_DRIVE_SPEED.div(Seconds.of(1)),
                SwerveConstants.DEFAULT_ROT_SPEED,
                SwerveConstants.DEFAULT_ROT_SPEED.div(Seconds.of(1)));
    }

    public static class VisionConstants {
        public static final String FRONT_LEFT_CAM_NAME = "Front Left Cam";
        public static final String FRONT_RIGHT_CAM_NAME = "Front Right Cam";
        public static final String BACK_CAM_NAME = "Back Cam";

        // Transforms from robot to cameras, (x forward, y left, z up), (roll, pitch, yaw)
        public static final Transform3d FRONT_LEFT_CAM_POS = new Transform3d(
            new Translation3d(SwerveConstants.MODULE_DISTANCE.div(2), SwerveConstants.MODULE_DISTANCE.div(-2), Inches.of(5)),
            new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.zero())
        );

        public static final Transform3d FRONT_RIGHT_CAM_POS = new Transform3d(
            new Translation3d(SwerveConstants.MODULE_DISTANCE.div(2), SwerveConstants.MODULE_DISTANCE.div(2), Inches.of(5)),
            new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.zero())
        );

        public static final Transform3d BACK_CAM_POS = new Transform3d(
            new Translation3d(Inches.zero(), SwerveConstants.MODULE_DISTANCE.div(-2), Inches.of(5)),
            new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.zero())
        );

        public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        public static final DoubleArrayTopic POSE_TOPIC = INST.getDoubleArrayTopic("/Vision/Estimated Pose");
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
                InchesPerSecond.ofNative(72), InchesPerSecond.ofNative(72).per(Second));

        // Setpoints
        public static final Distance L1_HEIGHT = Inches.of(0);
        public static final Distance L2_HEIGHT = Inches.of(0);
        public static final Distance L3_HEIGHT = Inches.of(0);
        public static final Distance L4_HEIGHT = Inches.of(0);
        public static final Distance INTAKE_HEIGHT = Inches.of(0);

        // Manual control (duty cycle)
        public static final double MANUAL_UP_SPEED = 0.3;
        public static final double MANUAL_DOWN_SPEED = -0.2;

        // Simulation
        public static final double GEAR_RATIO = 4;
        public static final Mass CARRIAGE_MASS = Kilograms.of(2);
        public static final Distance DRUM_RADIUS = Meters.of(0.05);
        public static final Distance MIN_HEIGHT = Meters.zero();
        public static final Distance MAX_HEIGHT = Meters.of(1.8);

        public static final Distance CANVAS_WIDTH = Inches.of(2);
        public static final Distance CANVAS_HEIGHT = Inches.of(42);
        public static final Translation2d ROOT = new Translation2d(Inches.of(5), Inches.of(10));
        
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
