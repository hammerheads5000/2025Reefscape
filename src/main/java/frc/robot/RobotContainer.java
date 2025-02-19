// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.AutoConstants.AUTO_DESCRIPTOR_TOPIC;
import static frc.robot.Constants.AutoConstants.CONSTRAINTS;
import static frc.robot.Constants.EndEffectorConstants.INTAKE_SPEED;
import static frc.robot.Constants.EndEffectorConstants.SCORE_SPEED;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AlignToPoseCommand;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.FullAutoCommand;
import frc.robot.commands.autos.TestPathCommands;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

@Logged
public class RobotContainer {
    // #region Controllers
    CommandXboxController primaryController = new CommandXboxController(0);
    // #endregion

    // #region Subsystems
    Swerve swerve = new Swerve();
    VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    SwerveTelemetry swerveTelemetry = new SwerveTelemetry();
    EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
    // #endregion

    // #region Commands
    TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, primaryController);

    AlignToPoseCommand moveToZero = new AlignToPoseCommand(Pose2d.kZero, SwerveConstants.SCORING_PID_X, SwerveConstants.SCORING_PID_Y, SwerveConstants.SCORING_PID_ANGLE, swerve);
    AlignToPoseCommand reefAlign = AlignToReefCommands.alignToReef(5, -1, swerve);
    // #endregion

    // #region Triggers
    Trigger fastSpeedTrigger = primaryController.rightTrigger();
    Trigger slowSpeedTrigger = primaryController.leftTrigger();

    Trigger resetFieldRelative = primaryController.y();

    Trigger elevatorUpTrigger = primaryController.povUp();
    Trigger elevatorDownTrigger = primaryController.povDown();
    Trigger elevatorIntakeTrigger = primaryController.povLeft();
    Trigger elevatorL2Trigger = primaryController.povRight();

    Trigger moveToZeroTrigger = primaryController.x();

    Trigger intakeTrigger = primaryController.rightBumper();
    Trigger reverseIntakeTrigger = primaryController.leftBumper();

    Trigger elevatorSysIdQuasistatic = primaryController.start(); // on the right
    Trigger elevatorSysIdDynamic = primaryController.back(); // on the left
    Trigger elevatorSysIdForward = primaryController.a();
    Trigger elevatorSysIdBack = primaryController.b();
    // #endregion

    StringEntry autoDescriptorSubscriber = AUTO_DESCRIPTOR_TOPIC.getEntry("");

    public RobotContainer() {
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
        swerve.setDefaultCommand(teleopSwerve);
        AlignToReefCommands.testReefPoses(); // publishes all reef target poses to networktables

        DriverStation.getAlliance().ifPresent(alliance -> {
            swerve.setOperatorPerspective(alliance == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg);
        });

        autoDescriptorSubscriber.set("");

        //elevatorSubsystem.disable();
        configureBindings();

    }

    private void configureBindings() {
        fastSpeedTrigger.whileTrue(teleopSwerve.fastSpeedCommand());
        slowSpeedTrigger.whileTrue(teleopSwerve.slowSpeedCommand());

        resetFieldRelative.onTrue(new InstantCommand(swerve::resetOdometry));

        elevatorUpTrigger.whileTrue(new InstantCommand(() -> elevatorSubsystem.setRotations(Rotations.of(elevatorSubsystem.getMotorRotations()).plus(Rotations.of(3)))));
        elevatorDownTrigger.whileTrue(new InstantCommand(() -> elevatorSubsystem.setRotations(Rotations.of(elevatorSubsystem.getMotorRotations()).minus(Rotations.of(3)))));
        // elevatorUpTrigger.whileTrue(elevatorSubsystem.moveUpManualCommand());
        // elevatorDownTrigger.whileTrue(elevatorSubsystem.moveDownManualCommand());
        elevatorIntakeTrigger.whileTrue(elevatorSubsystem.goToIntakePosCommand(false));
        elevatorL2Trigger.whileTrue(elevatorSubsystem.goToL2Command(false));

        moveToZeroTrigger.whileTrue(reefAlign);

        intakeTrigger.whileTrue(endEffectorSubsystem.forwardCommand(INTAKE_SPEED));
        reverseIntakeTrigger.whileTrue(endEffectorSubsystem.forwardCommand(SCORE_SPEED));

        elevatorSysIdQuasistatic.and(elevatorSysIdForward).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kForward));
        elevatorSysIdQuasistatic.and(elevatorSysIdBack).whileTrue(elevatorSubsystem.sysIdQuasistatic(Direction.kReverse));
        elevatorSysIdDynamic.and(elevatorSysIdForward).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kForward));
        elevatorSysIdDynamic.and(elevatorSysIdBack).whileTrue(elevatorSubsystem.sysIdDynamic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return new FullAutoCommand(autoDescriptorSubscriber.get(), swerve);
    }
}
