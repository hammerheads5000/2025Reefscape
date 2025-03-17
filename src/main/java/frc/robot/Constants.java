// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;

import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;

public class Constants {
    public static final CANBus CAN_FD_BUS = new CANBus("Bobby");
    public static final CANBus CAN_RIO_BUS = new CANBus("rio");

    public static final Time SIM_LOOP_PERIOD = Milliseconds.of(20);

    public static final NetworkTableInstance INST = NetworkTableInstance.getDefault();

    public static final double CONTROLLER_DEADBAND = .225;
    
    public static final Distance BUMPER_THICKNESS = Inches.of(4);

    public static class SwerveConstants {
        public static final LinearVelocity DEFAULT_DRIVE_SPEED = MetersPerSecond.of(2.5);//define later
        public static final AngularVelocity DEFAULT_ROT_SPEED = RotationsPerSecond.of(1.5);

        public static final LinearVelocity FAST_DRIVE_SPEED = MetersPerSecond.of(5);//find values later
        public static final AngularVelocity FAST_ROT_SPEED = RotationsPerSecond.of(4);

        public static final LinearVelocity SLOW_DRIVE_SPEED = MetersPerSecond.of(1);
        public static final AngularVelocity SLOW_ROT_SPEED = RotationsPerSecond.of(0.5);

        public static final LinearAcceleration MAX_TELEOP_ACCEL = MetersPerSecondPerSecond.of(10);
        public static final AngularVelocity MAX_MODULE_ROT_SPEED = RotationsPerSecond.of(5);

        // distance between modules on same side (front to back or left to right)
        private static final Distance MODULE_DISTANCE = Inches.of(23.75);

        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
                .withKP(200).withKI(5).withKD(8)
                .withKS(0.5).withKV(2.54).withKA(0.09)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(0.25).withKI(0.01).withKD(0.0)
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

        public static final LinearVelocity SPEED_AT_12V = MetersPerSecond.of(5); // maybe needs tuning

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        private static final double COUPLE_RATIO = 3.5714285714285716;

        private static final double DRIVE_GEAR_RATIO = 6.746031746031747;
        private static final double STEER_GEAR_RATIO = 21.428571428571427;
        private static final Distance WHEEL_RADIUS = Inches.of(1.875);

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
        public static final Matrix<N3, N1> VISION_STD_DEV_0M = VecBuilder.fill(0.06, 0.06, 0.5);
        public static final Matrix<N3, N1> VISION_STD_DEV_5M = VecBuilder.fill(2, 2, 3);

        public static final DriveRequestType DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;
        public static final SteerRequestType STEER_REQUEST_TYPE = SteerRequestType.MotionMagicExpo;
        
        public static final LinearVelocity LINEAR_VEL_DEADBAND = MetersPerSecond.of(0.02);
        public static final AngularVelocity ANGLULAR_VEL_DEADBAND = DegreesPerSecond.of(1);

        // Aligning to the reef to score coral

        // output: m/s, measure: m
        public static final ControlConstants SCORING_PID_X = new ControlConstants()
                .withPID(5, 0.5, 0.05).withTolerance(Inches.of(2).in(Meters), 0.1)
                .withProfile(DEFAULT_DRIVE_SPEED.in(MetersPerSecond), DEFAULT_DRIVE_SPEED.in(MetersPerSecond)/1);
        public static final ControlConstants SCORING_PID_Y = new ControlConstants()
                .withPID(4, 0.5, 0.05).withTolerance(Inches.of(1.0).in(Meters), 0.1)
                .withProfile(DEFAULT_DRIVE_SPEED.in(MetersPerSecond), DEFAULT_DRIVE_SPEED.in(MetersPerSecond)/1);

        public static ControlConstants ALGAE_PID_X = new ControlConstants(SCORING_PID_X)
                .withTolerance(Inches.of(2).in(Meters));
        public static ControlConstants ALGAE_PID_Y = new ControlConstants(SCORING_PID_Y)
                .withTolerance(Inches.of(4).in(Meters));
        
        // output: deg/s, measure: deg
        public static final ControlConstants SCORING_PID_ANGLE = new ControlConstants()
                .withPID(6, 2.0, 0.0).withTolerance(1.5);
        
        public static ControlConstants ALGAE_PID_ANGLE = new ControlConstants(SCORING_PID_ANGLE)
                .withTolerance(5);



        public static final Time ALIGN_TIME = Seconds.of(0.15); // amount to wait to make sure aligned

        // output: m/s, measure: m
        public static final PIDConstants PP_TRANSLATIONAL_PID = new PIDConstants(3, 0.5, 0.5);
        // output: rad/s, measure: rad
        public static final PIDConstants PP_ROTATIONAL_PID = new PIDConstants(2, 0, 0.5);
    }

    public static class AutoConstants {
        // Test Autos
        public static final PathConstraints CONSTRAINTS = new PathConstraints(
                SwerveConstants.FAST_DRIVE_SPEED,
                SwerveConstants.DEFAULT_DRIVE_SPEED.div(Seconds.of(0.675)),
                SwerveConstants.DEFAULT_ROT_SPEED,
                SwerveConstants.DEFAULT_ROT_SPEED.div(Seconds.of(1.5)));

        public static final Distance SIDE_DISTANCE = Meters.of(3);
    
        public static final Distance DISTANCE_TO_REEF = Inches.of(29 / 2).plus(BUMPER_THICKNESS);

        public static final Distance APPROACH_DISTANCE = Inches.of(15); // *extra* distance to reef when approaching
        public static final Distance PULL_DISTANCE = Inches.of(15);
        public static final Distance ELEVATOR_DEPLOY_DISTANCE = Inches.of(60);
        public static final Distance TRAVERSE_DISTANCE = Inches.of(40); // *extra* distance to reef when moving around to other side

        public static final LinearVelocity MIN_PATH_SPEED = MetersPerSecond.of(1);

        public static final Map<Character, Pair<Integer, Integer>> LETTER_TO_SIDE_AND_RELATIVE = Map.ofEntries(
                Map.entry(Character.valueOf('A'), new Pair<Integer, Integer>(0, 1)),
                Map.entry(Character.valueOf('B'), new Pair<Integer, Integer>(0, -1)),
                Map.entry(Character.valueOf('C'), new Pair<Integer, Integer>(5, 1)),
                Map.entry(Character.valueOf('D'), new Pair<Integer, Integer>(5, -1)),
                Map.entry(Character.valueOf('E'), new Pair<Integer, Integer>(4, 1)),
                Map.entry(Character.valueOf('F'), new Pair<Integer, Integer>(4, -1)),
                Map.entry(Character.valueOf('G'), new Pair<Integer, Integer>(3, 1)),
                Map.entry(Character.valueOf('H'), new Pair<Integer, Integer>(3, -1)),
                Map.entry(Character.valueOf('I'), new Pair<Integer, Integer>(2, 1)),
                Map.entry(Character.valueOf('J'), new Pair<Integer, Integer>(2, -1)),
                Map.entry(Character.valueOf('K'), new Pair<Integer, Integer>(1, 1)),
                Map.entry(Character.valueOf('L'), new Pair<Integer, Integer>(1, -1))
        );

        public static final StringTopic AUTO_DESCRIPTOR_TOPIC = INST.getStringTopic("Auto Descriptor");
        public static final StringTopic REEF_TELEOP_AUTO_TOPIC = INST.getStringTopic("Reef Descriptor");
        public static final StringTopic STATION_TELEOP_AUTO_TOPIC = INST.getStringTopic("Station Descriptor");
        public static final StringEntry REEF_TELEOP_AUTO_ENTRY = REEF_TELEOP_AUTO_TOPIC.getEntry("A4");
        public static final StringEntry STATION_TELEOP_AUTO_ENTRY = STATION_TELEOP_AUTO_TOPIC.getEntry("S0C");
    }

    public static class VisionConstants {
        public static final String FRONT_LEFT_CAM_NAME = "Arducam_OV9281_FL01";
        public static final String FRONT_RIGHT_CAM_NAME = "Arducam_OV9281_FR01";
        public static final String BACK_CAM_NAME = "Back Cam";

        public static final double MAX_AMBIGUITY = 0.2;

        // Transforms from robot to cameras, (x forward, y left, z up), (roll, pitch, yaw)
        public static final Transform3d FRONT_LEFT_CAM_POS = new Transform3d(
            new Translation3d(SwerveConstants.MODULE_DISTANCE.div(2), SwerveConstants.MODULE_DISTANCE.div(2), Inches.of(10)),
            new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-45))
        );

        public static final Transform3d FRONT_RIGHT_CAM_POS = new Transform3d(
            new Translation3d(SwerveConstants.MODULE_DISTANCE.div(2), SwerveConstants.MODULE_DISTANCE.div(-2), Inches.of(10)),
            new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.of(45))
        );

        public static final Transform3d BACK_CAM_POS = new Transform3d(
            new Translation3d(Inches.zero(), SwerveConstants.MODULE_DISTANCE.div(-2), Inches.of(5)),
            new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.zero())
        );

        public static final PoseStrategy POSE_STRATEGY = PoseStrategy.AVERAGE_BEST_TARGETS;

        public static final DoubleArrayTopic POSE_TOPIC = INST.getDoubleArrayTopic("/Vision/Estimated Pose");
    }

    public static class ElevatorConstants {
        // Motors
        public static final int MOTOR_1_ID = 12; // fd bus
        public static final int MOTOR_2_ID = 13; // fd bus
        public static final int ENCODER_ID = 5; // fd bus
        
        // Motor Configs
        public static final double GEAR_RATIO = 16;

        public static final boolean OPPOSE_FOLLOWER = true;

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(70));

        public static final MotorOutputConfigs OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(ENCODER_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRotorToSensorRatio(GEAR_RATIO);

        public static final Slot0Configs GAINS = new Slot0Configs()
                .withKP(20).withKI(15).withKD(1)
                .withKV(0.74).withKA(0.0)
                .withKS(0.09).withKG(0.48);

        public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
                .withMotionMagicExpo_kV(Volts.of(0.6).per(RotationsPerSecond))
                .withMotionMagicExpo_kA(Volts.of(0.75).per(RotationsPerSecondPerSecond));

        public static final TalonFXConfiguration MOTOR_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                .withMotorOutput(OUTPUT_CONFIGS)
                .withFeedback(FEEDBACK_CONFIGS)
                .withSlot0(GAINS)
                .withMotionMagic(MOTION_MAGIC_CONFIGS);

        public static final MagnetSensorConfigs ENCODER_CONFIGS = new MagnetSensorConfigs()
                .withMagnetOffset(-0.258)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);


        public static final Angle TOLERANCE = Rotations.of(0.05);

        public static final DoubleTopic SETPOINT_TOPIC = INST.getTable("Elevator").getDoubleTopic("ElevatorSetpoint_rotations");

        
        // Manual control (duty cycle)
        public static final double MANUAL_UP_SPEED = 0.3;
        public static final double MANUAL_DOWN_SPEED = -0.2;

        public static final Current STALL_CURRENT = Amps.of(55);
        
        public static final Mass CARRIAGE_MASS = Ounces.of(50.884);
        public static final Distance DRUM_RADIUS = Inches.of(1.756);
        public static final Distance MIN_HEIGHT = Meters.zero();
        public static final Distance MAX_HEIGHT = Inches.of(72.154); // from base plate
        public static final Distance MIN_LASERCAN_DISTANCE = Inches.of(0.5);
        public static final double HEIGHT_CHANGE_PER_LASERCAN_DISTANCE = -3;
        public static final Per<AngleUnit, DistanceUnit> MOTOR_ROTATIONS_PER_LASERCAN = Rotations.of(-152).div(Meters.of(1));
        public static final Angle MOTOR_ROTATIONS_AT_LASERCAN_0 = Rotations.of(91);
        public static final Per<DistanceUnit, AngleUnit> HEIGHT_PER_MOTOR_ROTATIONS = Inches.of(0.82).div(Rotations.of(1));
        public static final Distance LASERCAN_RELIABILITY_MIN = Meters.of(0.45);

        public static final Distance CANVAS_WIDTH = Inches.of(2);
        public static final Distance CANVAS_HEIGHT = Inches.of(42);
        public static final Translation2d ROOT = new Translation2d(Inches.of(7), Inches.of(3.875));
        
        public static final RangingMode LASERCAN_RANGING_MODE = RangingMode.SHORT;
        public static final RegionOfInterest REGION_OF_INTEREST = new RegionOfInterest(8, 3, 6, 6);

        // Setpoints
        public static final Angle L1_HEIGHT = Rotations.of(1.325);
        public static final Angle L2_HEIGHT = Rotations.of(1.975);
        public static final Angle L3_HEIGHT = Rotations.of(2.83);
        public static final Angle L4_HEIGHT = Rotations.of(4.3675);
        public static final Angle INTAKE_HEIGHT = Rotations.of(0.13);
        
        public static final Angle INTAKE_JITTER_AMOUNT = Rotations.of(0.025);
        public static final Time INTAKE_JITTER_PERIOD = Seconds.of(0.75);
    }

    public static class EndEffectorConstants {
        // Motors
        public static final int MOTOR_LEFT_ID = 30;
        public static final int MOTOR_RIGHT_ID = 31;

        public static final MotorOutputConfigs MOTOR_LEFT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);
        public static final MotorOutputConfigs MOTOR_RIGHT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);
        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(10));

        // Lidar
        public static final int FRONT_LIDAR_ID = 9;
        public static final int BACK_LIDAR_ID = 8;

        // Speed (duty cycle)
        public static final double INTAKE_SPEED = 0.25;
        public static final double SLOW_INTAKE_SPEED = 0.15;
        public static final double SCORE_SPEED = 0.15;
        public static final double FAST_TROUGH_SPEED = 0.6;
        public static final double SLOW_TROUGH_SPEED = 0.25;
    }

    public static class ClimberConstants {
        public static final int MOTOR_ID = 13;
        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(40));
        
        public static final MotorOutputConfigs OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final Voltage CLIMB_SPEED = Volts.of(10);
        public static final Voltage BREAK_SPEED = Volts.of(6);
        public static final Voltage RELEASE_SPEED = Volts.of(-6);

        public static final double GEAR_RATIO = (46.0 / 26) * (54.0 / 20) * 100;
        public static final Angle BREAK_ANGLE = Degrees.of(260);
        public static final Angle ALIGN_ANGLE = Degrees.of(115);

        public static final Current STALL_CURRENT = Amps.of(50);
    }

    public static class LightsConstants {
        public static final Distance LED_SPACING = Meters.of(1).div(144);

        public static final int PWM_PORT = 0; // TODO define

        public static final int[] LED_SEPARATIONS = {90, 153, 175, 255, 335, 354, 415, 500};

        public static final int HIGH_TIME_0_NS = 400;
        public static final int HIGH_TIME_1_NS = 900;
        public static final int LOW_TIME_0_NS = 900;
        public static final int LOW_TIME_1_NS = 600;
        public static final int SYNC_TIME_US = 280;
        public static final ColorOrder COLOR_ORDER = ColorOrder.kGRB;

        public static final Dimensionless BRIGHTNESS = Percent.of(35);
        
        public static final LEDPattern RAINBOW = LEDPattern.rainbow(255, 128)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.15), LED_SPACING);

        public static final Color PATH_FOLLOWING_COLOR = Color.kBlue;
        public static final Color ALIGNMENT_COLOR = Color.kPink;
        public static final Color ALIGNED_COLOR = Color.kMagenta;
        public static final Color INTAKE_COLOR = Color.kPurple;
        public static final Color ALGAE_COLOR = Color.kMediumAquamarine;
        public static final LEDPattern IDLE_PATTERN = RAINBOW;

        public static final Color HAS_TARGET_COLOR = Color.kDarkBlue;
        public static final Color HAS_NO_TARGET_COLOR = Color.kLightBlue;
        public static final Color NO_VISION_COLOR = Color.kOrange;
        public static final Color LOW_BATTERY_COLOR = Color.kRed;

        public static final Distance MAX_VISION_DISTANCE = Meters.of(1);
        public static final Voltage LOW_BATTERY_VOLTAGE = Volts.of(12.4);

        public static final Time FADE_START = Seconds.of(1);
        public static final Time FADE_DURATION = Seconds.of(3);
    }

    public static class FieldConstants {
        public static final AprilTagFieldLayout APRIL_TAGS = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeAndyMark);

        public static final Pose2d LEFT_CORAL_STATION = new Pose2d(
                Meters.of(1.16), Meters.of(7.013),
                Rotation2d.fromDegrees(125));

        public static final Pose2d RIGHT_CORAL_STATION = new Pose2d(
                Meters.of(1.16), Meters.of(1.02),
                Rotation2d.fromDegrees(-125));

        public static final Pose2d STATION_0 = APRIL_TAGS.getTagPose(12).get().toPose2d();
        public static final Pose2d STATION_1 = APRIL_TAGS.getTagPose(13).get().toPose2d();
        
        
        public static final Distance STATION_APPROACH_DISTANCE = Inches.of(24);
        public static final Distance SIDE_STATION_OFFSET = Inches.of(29).plus(BUMPER_THICKNESS).div(2);
        
        // Pose at midpoint between tags 18 and 21 (which are opposite on blue reef)
        public static final Translation2d REEF_CENTER_BLUE = APRIL_TAGS.getTagPose(18).get().toPose2d().getTranslation()
        .plus(APRIL_TAGS.getTagPose(21).get().toPose2d().getTranslation()).div(2);
        
        // Pose at midpoint between tags 10 and 7 (which are opposite on red reef)
        public static final Translation2d REEF_CENTER_RED = APRIL_TAGS.getTagPose(10).get().toPose2d().getTranslation()
        .plus(APRIL_TAGS.getTagPose(7).get().toPose2d().getTranslation()).div(2);
        
        // Distance from center of robot to center of reef
        // Found by taking distance from tag 18 to center and adding offset from reef
        public static final Distance REEF_APOTHEM = Meters.of(
                APRIL_TAGS.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER_BLUE))
                .plus(AutoConstants.DISTANCE_TO_REEF);
                
        // translation to move from centered on a side to scoring position for the left branch
        public static final Translation2d CENTERED_TO_LEFT_BRANCH = new Translation2d(Meters.of(0),
                Inches.of(12.94 / 2));

        public static final Distance L1_SIDE_DISTANCE = Inches.of(18);
        public static final double L1_RELATIVE_POS = L1_SIDE_DISTANCE.div(CENTERED_TO_LEFT_BRANCH.getMeasureY()).magnitude();
    }
}
