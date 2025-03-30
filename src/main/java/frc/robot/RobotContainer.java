// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.AutoConstants.AUTO_DESCRIPTOR_TOPIC;
import static frc.robot.Constants.AutoConstants.REEF_TELEOP_AUTO_ENTRY;
import static frc.robot.Constants.AutoConstants.STATION_TELEOP_AUTO_ENTRY;
import static frc.robot.Constants.EndEffectorConstants.INTAKE_SPEED;
import static frc.robot.Constants.FieldConstants.FIELD;
import static frc.robot.Constants.LightsConstants.IDLE_PATTERN;

import java.util.Map;
import java.util.Set;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.commands.DisabledLightsCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.ApproachCoralStationCommands;
import frc.robot.commands.autos.FullAutoCommand;
import frc.robot.commands.autos.RemoveAlgaeCommand;
import frc.robot.commands.autos.SweepCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

@Logged
public class RobotContainer {
    // #region Controllers
    CommandXboxController primaryController = new CommandXboxController(0);
    CommandGenericHID buttonBoardReef = new CommandGenericHID(1);
    CommandGenericHID buttonBoardOther = new CommandGenericHID(2);
    // #endregion

    PowerDistribution pdh = new PowerDistribution();

    StringEntry autoDescriptorEntry = AUTO_DESCRIPTOR_TOPIC.getEntry("");

    // #region Subsystems
    Swerve swerve = new Swerve();
    VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    SwerveTelemetry swerveTelemetry = new SwerveTelemetry();
    EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
    LightsSubsystem lightsSubsystem = new LightsSubsystem();
    ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    // #endregion

    // #region Commands
    TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, primaryController);

    AlignToPoseCommand moveToZero = new AlignToPoseCommand(Pose2d.kZero, SwerveConstants.SCORING_PID_TRANSLATION, SwerveConstants.SCORING_PID_ANGLE, swerve);

    DisabledLightsCommand disabledLightsCommand = new DisabledLightsCommand(lightsSubsystem, visionSubsystem);
    AlignToPoseCommand reefAlign = AlignToReefCommands.alignToReef(0, 1, swerve);

    Command rumbleCommand = Commands.startEnd(
            () -> primaryController.setRumble(RumbleType.kBothRumble, Constants.CONTROLLER_RUMBLE), 
            () -> primaryController.setRumble(RumbleType.kBothRumble, 0));

    Command reefCommand = Commands.defer(
            () -> new FullAutoCommand(REEF_TELEOP_AUTO_ENTRY.get(), swerve, elevatorSubsystem,
                    endEffectorSubsystem, lightsSubsystem),
            Set.of(swerve, elevatorSubsystem)).handleInterrupt(() -> lightsSubsystem.setPattern(IDLE_PATTERN))
            .andThen(rumbleCommand.asProxy().withTimeout(Constants.SCORE_RUMBLE_TIME));

    Command stationCommand = Commands.defer(
            () -> new FullAutoCommand(STATION_TELEOP_AUTO_ENTRY.get(), swerve, elevatorSubsystem,
                    endEffectorSubsystem, lightsSubsystem),
            Set.of(swerve, elevatorSubsystem)).handleInterrupt(() -> lightsSubsystem.setPattern(IDLE_PATTERN));

    Command algaeCommand = Commands.defer(
            () -> new RemoveAlgaeCommand(swerve, elevatorSubsystem, lightsSubsystem),
            Set.of(swerve, elevatorSubsystem)).handleInterrupt(() -> lightsSubsystem.setPattern(IDLE_PATTERN));

    Command sweepCommand = Commands.defer(
        () -> new SweepCommand(swerve), Set.of(swerve));

    Map<Character, Command> ELEVATOR_COMMANDS = Map.ofEntries(
            Map.entry('1', elevatorSubsystem.goToL1Command(false)),
            Map.entry('2', elevatorSubsystem.goToL2Command(false)),
            Map.entry('3', elevatorSubsystem.goToL3Command(false)),
            Map.entry('4', elevatorSubsystem.goToL4Command(false)));

    Command elevatorCommand = Commands.defer(
            () -> ELEVATOR_COMMANDS.get(REEF_TELEOP_AUTO_ENTRY.get().charAt(1)), Set.of(elevatorSubsystem));
    // #endregion

    // #region Triggers
    Trigger fastSpeedTrigger = primaryController.rightTrigger();
    Trigger slowSpeedTrigger = primaryController.leftTrigger();

    Trigger elevatorUpTrigger = primaryController.povUp();
    Trigger elevatorDownTrigger = primaryController.povDown();
    Trigger elevatorIntakeTrigger = primaryController.povLeft();
    // Trigger elevatorL2Trigger = primaryController.povRight();

    // Trigger moveToZeroTrigger = primaryController.x();

    Trigger intakeTrigger = primaryController.rightBumper();
    Trigger reverseIntakeTrigger = primaryController.leftBumper();

//     Trigger sysIdQuasistatic = primaryController.start(); // on the right
//     Trigger sysIdDynamic = primaryController.back(); // on the left
//     Trigger sysIdForward = primaryController.a();
//     Trigger sysIdBack = primaryController.b();

    Trigger reefTrigger = primaryController.a();//.and(sysIdQuasistatic.or(sysIdDynamic).negate());
    Trigger stationTrigger = primaryController.b();//.and(sysIdQuasistatic.or(sysIdDynamic).negate());
    Trigger algaeTrigger = primaryController.y();
    Trigger sweepTrigger = primaryController.x();
    Trigger elevatorTrigger = buttonBoardOther.button(1);
    //Trigger elevatorIntakeTrigger = buttonBoardOther.button(2);
    Trigger climbTrigger = primaryController.start();
    Trigger unclimbTrigger = primaryController.back();

    Map<Integer, Character> BUTTON_TO_REEF = Map.ofEntries(
            Map.entry(5, 'A'),
            Map.entry(7, 'B'),
            Map.entry(9, 'C'),
            Map.entry(11, 'D'),
            Map.entry(12, 'E'),
            Map.entry(10, 'F'),
            Map.entry(8, 'G'),
            Map.entry(6, 'H'),
            Map.entry(4, 'I'),
            Map.entry(2, 'J'),
            Map.entry(1, 'K'),
            Map.entry(3, 'L'));

    Map<Integer, String> BUTTON_TO_STATION = Map.ofEntries(
            Map.entry(3, "S1L"),
            Map.entry(4, "S1C"),
            Map.entry(5, "S1R"),
            Map.entry(6, "S0L"),
            Map.entry(7, "S0C"),
            Map.entry(8, "S0R"));

    Trigger[] reefTriggers = new Trigger[12];
    Trigger[] levelTriggers = new Trigger[] {
            buttonBoardOther.button(9),
            buttonBoardOther.button(10),
            buttonBoardOther.button(11),
            buttonBoardOther.button(12)
    };

    Trigger[] stationTriggers = new Trigger[6];

    Trigger hasCoralTrigger = new Trigger(() -> !endEffectorSubsystem.getIntakeLidar() || !endEffectorSubsystem.getBackLidar());
    // #endregion

    public RobotContainer() {
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
        swerve.setDefaultCommand(teleopSwerve);
        AlignToReefCommands.testReefPoses(); // publishes all reef target poses to networktables
        ApproachCoralStationCommands.testStationPoses();

        configureAlliance();

        autoDescriptorEntry.set("");
        REEF_TELEOP_AUTO_ENTRY.set("A4");
        STATION_TELEOP_AUTO_ENTRY.set("S0");
        SmartDashboard.putData(FIELD);

        // elevatorSubsystem.disable();
        configureBindings();
    }

    public void configureAlliance() {
        DriverStation.getAlliance().ifPresent(alliance -> {
                swerve.setOperatorPerspective(alliance == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg);
        });
    }

    private void configureBindings() {
        fastSpeedTrigger.whileTrue(teleopSwerve.fastSpeedCommand());
        slowSpeedTrigger.whileTrue(teleopSwerve.slowSpeedCommand());

        // elevatorUpTrigger.whileTrue(new InstantCommand(() ->
        // elevatorSubsystem.setRotations(elevatorSubsystem.getMotorRotations().plus(Rotations.of(3)))));
        // elevatorDownTrigger.whileTrue(new InstantCommand(() ->
        // elevatorSubsystem.setRotations(elevatorSubsystem.getMotorRotations().minus(Rotations.of(3)))));
        elevatorUpTrigger.whileTrue(elevatorSubsystem.moveUpManualCommand());
        elevatorDownTrigger.whileTrue(elevatorSubsystem.moveDownManualCommand());
        elevatorIntakeTrigger.whileTrue(elevatorSubsystem.goToIntakePosCommand(false));
        // elevatorL2Trigger.whileTrue(elevatorSubsystem.goToL2Command(false));

        // moveToZeroTrigger.whileTrue(reefAlign);

        intakeTrigger.whileTrue(endEffectorSubsystem.forwardCommand(INTAKE_SPEED));
        reverseIntakeTrigger.whileTrue(endEffectorSubsystem.forwardCommand(-INTAKE_SPEED));

        reefTrigger.whileTrue(reefCommand);
        stationTrigger.whileTrue(stationCommand);
        algaeTrigger.whileTrue(algaeCommand);
        sweepTrigger.whileTrue(sweepCommand);

        climbTrigger.whileTrue(climberSubsystem.climbCommand());
        unclimbTrigger.whileTrue(climberSubsystem.reverseCommand());

        // sysIdQuasistatic.and(sysIdForward).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        // sysIdQuasistatic.and(sysIdBack).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));
        // sysIdDynamic.and(sysIdForward).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        // sysIdDynamic.and(sysIdBack).whileTrue(swerve.sysIdDynamic(Direction.kReverse));

        for (int i = 0; i < reefTriggers.length; i++) {
            reefTriggers[i] = buttonBoardReef.button(i + 1);

            final char branch = BUTTON_TO_REEF.get(i + 1);
            reefTriggers[i]
                    .onTrue(new InstantCommand(() -> setTeleopAutoDescriptorLetter(branch)).ignoringDisable(true));
        }

        for (int i = 0; i < levelTriggers.length; i++) {
            final int level = i + 1;
            levelTriggers[i]
                    .onTrue(new InstantCommand(() -> setTeleopAutoDescriptorLevel(level)).ignoringDisable(true));
        }

        for (int i = 0; i < stationTriggers.length; i++) {
            final int button = i + 3;
            stationTriggers[i] = buttonBoardOther.button(button);
            stationTriggers[i]
                    .onTrue(new InstantCommand(() -> setTeleopAutoDescriptorStation(BUTTON_TO_STATION.get(button))).ignoringDisable(true));
        }

        hasCoralTrigger.whileTrue(rumbleCommand.asProxy()); // rumble while has coral
    }

    private void setTeleopAutoDescriptorLetter(char letter) {
        String descriptor = REEF_TELEOP_AUTO_ENTRY.get();
        REEF_TELEOP_AUTO_ENTRY.set(letter + descriptor.substring(1));
    }

    private void setTeleopAutoDescriptorLevel(int level) {
        String descriptor = REEF_TELEOP_AUTO_ENTRY.get();
        REEF_TELEOP_AUTO_ENTRY.set(descriptor.substring(0, 1) + level);
    }
    
    private void setTeleopAutoDescriptorStation(String station) {
        STATION_TELEOP_AUTO_ENTRY.set(station);
    }

    public Command getAutonomousCommand() {
        return new FullAutoCommand(autoDescriptorEntry.get(), swerve, elevatorSubsystem, endEffectorSubsystem,
                lightsSubsystem);
    }
}
