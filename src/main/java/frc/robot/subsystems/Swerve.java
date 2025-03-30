package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.SIM_LOOP_PERIOD;
import static frc.robot.Constants.SwerveConstants.*;

import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants.BackLeft;
import frc.robot.Constants.SwerveConstants.BackRight;
import frc.robot.Constants.SwerveConstants.FrontLeft;
import frc.robot.Constants.SwerveConstants.FrontRight;

public class Swerve extends SubsystemBase {
    private Notifier simNotifier = null;
    private double lastSimTime;

    private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private SwerveRequest.FieldCentric fieldCentricRequest;
    private SwerveRequest.RobotCentric robotCentricRequest;
    private SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds;
    private SwerveRequest.SwerveDriveBrake brakeRequest;

    private SwerveSetpointGenerator swerveSetpointGenerator;
    private SwerveSetpoint previouSetpoint;
    @Logged
    private ChassisSpeeds ppChassisSpeeds;

    // #region SysId Setup
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(12), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> drivetrain.setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> drivetrain.setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        drivetrain.setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
    // #endregion

    public Swerve() {
        drivetrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(TalonFX::new, TalonFX::new, CANcoder::new,
                DRIVETRAIN_CONSTANTS, ODOMETRY_UPDATE_FREQ.in(Hertz), ODOMETRY_STD_DEV, VISION_STD_DEV_0M,
                FrontLeft.MODULE_CONSTANTS, FrontRight.MODULE_CONSTANTS,
                BackLeft.MODULE_CONSTANTS, BackRight.MODULE_CONSTANTS);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        fieldCentricRequest = new SwerveRequest.FieldCentric()
                .withDeadband(LINEAR_VEL_DEADBAND).withRotationalDeadband(ANGLULAR_VEL_DEADBAND)
                .withDriveRequestType(DRIVE_REQUEST_TYPE).withSteerRequestType(STEER_REQUEST_TYPE);
        robotCentricRequest = new SwerveRequest.RobotCentric()
                .withDeadband(LINEAR_VEL_DEADBAND).withRotationalDeadband(ANGLULAR_VEL_DEADBAND)
                .withDriveRequestType(DRIVE_REQUEST_TYPE).withSteerRequestType(STEER_REQUEST_TYPE);
        pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
                .withDriveRequestType(DRIVE_REQUEST_TYPE).withSteerRequestType(STEER_REQUEST_TYPE);
    
        configPathPlanner();
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SIM_LOOP_PERIOD.in(Seconds));
    }

    // #region SysId Commands
    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
    // #endregion

    /**
     * Sets the perspective of the operator for field-centric operations
     * @param perspective Rotation representing perspective:
     *  <ul>
     *  <li> Blue Alliance: 0 deg
     *  <li> Red Alliance: 180 deg
     *  </ul>
     */
    public void setOperatorPerspective(Rotation2d perspective) {
        SmartDashboard.putNumber("VLKNEIO", perspective.getDegrees());
        drivetrain.setOperatorPerspectiveForward(perspective);
    }

    public void stop() {
        drivetrain.setControl(fieldCentricRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    public void driveFieldCentric(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rotVel) {
        drivetrain.setControl(fieldCentricRequest.withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(rotVel));
    }

    public void driveFieldCentricAbsolute(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rotVel) {
        drivetrain.setControl(fieldCentricRequest.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(rotVel));
    }

    public void driveRobotCentric(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rotVel) {
        drivetrain.setControl(robotCentricRequest
                .withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(rotVel));
    }

    public void applyChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        previouSetpoint = swerveSetpointGenerator.generateSetpoint(previouSetpoint, chassisSpeeds, 0.02);
        ppChassisSpeeds = previouSetpoint.robotRelativeSpeeds();
        drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(previouSetpoint.robotRelativeSpeeds()));
    }

    public void applyChassisSpeedsWithFeedforwards(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
        previouSetpoint = swerveSetpointGenerator.generateSetpoint(previouSetpoint, chassisSpeeds, 0.02);
        ppChassisSpeeds = previouSetpoint.robotRelativeSpeeds();
        drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(previouSetpoint.robotRelativeSpeeds())
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY()));
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return drivetrain.getState().Speeds;
    }

    public ChassisSpeeds getFieldSpeeds() {
        SwerveDriveState state = drivetrain.getState();
        return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    }

    /**
     * Applies vision measurement
     * 
     * @param estimatedRobotPose Returned from
     *                           {@link org.photonvision.PhotonPoseEstimator#update()
     *                           photonPoseEstimator.update()}
     *                           Contains a Pose3d and a timestamp
     */
    public void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose) {
        drivetrain.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds));
    }

    /**
     * Applies vision measurement with standard deviations
     * 
     * @param estimatedRobotPose Returned from
     *                           {@link org.photonvision.PhotonPoseEstimator#update()
     *                           photonPoseEstimator.update()}
     *                           Contains a Pose3d and a timestamp
     * @param stdDevs            Standard deviations matrix for confidence
     */
    public void addVisionMeasurement(EstimatedRobotPose estimatedRobotPose, Matrix<N3, N1> stdDevs) {
        drivetrain.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds), stdDevs);
    }

    /**
     * Applies vision measurement from NetworkTables data
     * 
     * @param poseArray double array storing the data of a Pose2d of format {x
     *                  (meters), y (meters), rotation (radians)}
     * @param timestamp FPGA timestamp in microseconds
     */
    public void addVisionMeasurement(double[] poseArray, long timestamp) {
        // converts array of format {x (m), y (m), rotation (rad)} to Pose2d
        Pose2d pose = new Pose2d(poseArray[0], poseArray[1], new Rotation2d(poseArray[2]));
        double timestampSeconds = Microseconds.of(timestamp).in(Seconds);
        timestampSeconds = Utils.fpgaToCurrentTime(timestampSeconds);
        drivetrain.addVisionMeasurement(pose, timestampSeconds);
    }

    public void resetOdometry(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    public void resetOdometry() {
        drivetrain.resetPose(Pose2d.kZero);
    }

    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        drivetrain.registerTelemetry(telemetryFunction);
    }

    private RobotConfig getPPConfig() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        
        return config;
    }

    private void configPathPlanner() {
        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            this::applyChassisSpeedsWithFeedforwards,
            new PPHolonomicDriveController(
                PP_TRANSLATIONAL_PID,
                PP_ROTATIONAL_PID
            ),
            getPPConfig(),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );

        swerveSetpointGenerator = new SwerveSetpointGenerator(getPPConfig(), MAX_MODULE_ROT_SPEED.in(RadiansPerSecond));
        previouSetpoint = new SwerveSetpoint(getChassisSpeeds(), drivetrain.getState().ModuleStates, DriveFeedforwards.zeros(4));
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateSimState(0.02, RobotController.getBatteryVoltage());
    }
}
