// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.AutoConstants.AUTO_DESCRIPTOR_TOPIC;
import static frc.robot.Constants.AutoConstants.TELEOP_AUTO_DESCRIPTOR_TOPIC;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.commands.DisabledLightsCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.ApproachCoralStationCommands;
import frc.robot.commands.autos.FullAutoCommand;
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
    
    // #region Subsystems
    Swerve swerve = new Swerve();
    VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    SwerveTelemetry swerveTelemetry = new SwerveTelemetry();
    EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
    ClimberSubsystem climberSubsystem = new ClimberSubsystem(pdh);
    LightsSubsystem lightsSubsystem = new LightsSubsystem();
    // #endregion

    StringEntry autoDescriptorEntry = AUTO_DESCRIPTOR_TOPIC.getEntry("");
    StringEntry teleopAutoDescriptorEntry = TELEOP_AUTO_DESCRIPTOR_TOPIC.getEntry("");

    // #region Commands
    TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, primaryController);

    AlignToPoseCommand moveToZero = new AlignToPoseCommand(Pose2d.kZero, SwerveConstants.SCORING_PID_X, SwerveConstants.SCORING_PID_Y, SwerveConstants.SCORING_PID_ANGLE, swerve);

    Command disabledLightsCommand = new DisabledLightsCommand(lightsSubsystem, visionSubsystem).ignoringDisable(true);
    AlignToPoseCommand reefAlign = AlignToReefCommands.alignToReef(0, 1, swerve);

    Command approachAndScoreCommand = Commands.defer(
            () -> new FullAutoCommand(teleopAutoDescriptorEntry.get(), swerve, elevatorSubsystem,
                    endEffectorSubsystem, lightsSubsystem),
            Set.of(swerve, elevatorSubsystem, endEffectorSubsystem, lightsSubsystem));
    // #endregion
    
    // #region Triggers
    Trigger fastSpeedTrigger = primaryController.rightTrigger();
    Trigger slowSpeedTrigger = primaryController.leftTrigger();
    
    Trigger resetFieldRelative = primaryController.y();

    // Trigger elevatorUpTrigger = primaryController.povUp();
    // Trigger elevatorDownTrigger = primaryController.povDown();
    // Trigger elevatorIntakeTrigger = primaryController.povLeft();
    // Trigger elevatorL2Trigger = primaryController.povRight();
    Trigger climbTrigger = primaryController.povUp();
    Trigger reverseClimbTrigger = primaryController.povDown();

    Trigger moveToZeroTrigger = primaryController.x();

    Trigger intakeTrigger = primaryController.rightBumper();
    Trigger reverseIntakeTrigger = primaryController.leftBumper();

    Trigger elevatorSysIdQuasistatic = primaryController.start(); // on the right
    Trigger elevatorSysIdDynamic = primaryController.back(); // on the left
    Trigger elevatorSysIdForward = primaryController.a();
    Trigger elevatorSysIdBack = primaryController.b();

    Trigger approachAndScoreTrigger = primaryController.a();
    
    Map<Integer, Character> BUTTON_TO_REEF = Map.ofEntries(
        Map.entry(4, 'A'),
        Map.entry(6, 'B'),
        Map.entry(8, 'C'),
        Map.entry(10, 'D'),
        Map.entry(11, 'E'),
        Map.entry(9, 'F'),
        Map.entry(7, 'G'),
        Map.entry(5, 'H'),
        Map.entry(3, 'I'),
        Map.entry(1, 'J'),
        Map.entry(0, 'K'),
        Map.entry(2, 'L')
    );

    Trigger[] reefTriggers = new Trigger[12];
    Trigger[] levelTriggers = new Trigger[] {
        buttonBoardOther.button(8),
        buttonBoardOther.button(9),
        buttonBoardOther.button(10),
        buttonBoardOther.button(11)
    };

    Trigger station0Trigger = buttonBoardOther.button(6);
    Trigger station1Trigger = buttonBoardOther.button(3);
    // #endregion


    public RobotContainer() {
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
        swerve.setDefaultCommand(teleopSwerve);
        AlignToReefCommands.testReefPoses(); // publishes all reef target poses to networktables
        ApproachCoralStationCommands.testStationPoses();

        DriverStation.getAlliance().ifPresent(alliance -> {
            swerve.setOperatorPerspective(alliance == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg);
        });

        autoDescriptorEntry.set("");
        teleopAutoDescriptorEntry.set("");

        //elevatorSubsystem.disable();
        climberSubsystem.latchIntake();
        configureBindings();
    }

    private void configureBindings() {
        fastSpeedTrigger.whileTrue(teleopSwerve.fastSpeedCommand());
        slowSpeedTrigger.whileTrue(teleopSwerve.slowSpeedCommand());

        resetFieldRelative.onTrue(new InstantCommand(swerve::resetOdometry));

        // elevatorUpTrigger.whileTrue(new InstantCommand(() -> elevatorSubsystem.setRotations(elevatorSubsystem.getMotorRotations().plus(Rotations.of(3)))));
        // elevatorDownTrigger.whileTrue(new InstantCommand(() -> elevatorSubsystem.setRotations(elevatorSubsystem.getMotorRotations().minus(Rotations.of(3)))));
        // elevatorUpTrigger.whileTrue(elevatorSubsystem.moveUpManualCommand());
        // elevatorDownTrigger.whileTrue(elevatorSubsystem.moveDownManualCommand());
        // elevatorIntakeTrigger.whileTrue(elevatorSubsystem.goToIntakePosCommand(false));
        // elevatorL2Trigger.whileTrue(elevatorSubsystem.goToL2Command(false));
        climbTrigger.whileTrue(climberSubsystem.climbCommand());
        reverseClimbTrigger.whileTrue(climberSubsystem.reverseCommand());

        moveToZeroTrigger.whileTrue(reefAlign);

        intakeTrigger.whileTrue(endEffectorSubsystem.intakeCommand());
        reverseIntakeTrigger.whileTrue(endEffectorSubsystem.scoreCommand());

        approachAndScoreTrigger.whileTrue(approachAndScoreCommand);

        elevatorSysIdQuasistatic.and(elevatorSysIdForward).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kForward));
        elevatorSysIdQuasistatic.and(elevatorSysIdBack).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kReverse));
        elevatorSysIdDynamic.and(elevatorSysIdForward).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kForward));
        elevatorSysIdDynamic.and(elevatorSysIdBack).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kReverse));

        for(int i = 0; i < reefTriggers.length; i++) {
            reefTriggers[i] = buttonBoardReef.button(i);

            final char branch = BUTTON_TO_REEF.get(i);
            reefTriggers[i].onTrue(new InstantCommand(() -> setTeleopAutoDescriptorLetter(branch)));
        }

        for (int i = 0; i < levelTriggers.length; i++) {
            final int level = i + 1;
            levelTriggers[i].onTrue(new InstantCommand(() -> setTeleopAutoDescriptorLevel(level)));
        }

        station0Trigger.onTrue(new InstantCommand(() -> teleopAutoDescriptorEntry.set("S0")));
        station1Trigger.onTrue(new InstantCommand(() -> teleopAutoDescriptorEntry.set("S1")));
    }

    private void setTeleopAutoDescriptorLetter(char letter) {
        String descriptor = teleopAutoDescriptorEntry.get();
        teleopAutoDescriptorEntry.set(letter + descriptor.substring(1));
    }

    private void setTeleopAutoDescriptorLevel(int level) {
        String descriptor = teleopAutoDescriptorEntry.get();
        teleopAutoDescriptorEntry.set(descriptor.substring(0, 1) + level);
    }

    public Command getAutonomousCommand() {
        return new FullAutoCommand(autoDescriptorEntry.get(), swerve, elevatorSubsystem, endEffectorSubsystem, lightsSubsystem);
    }
}
