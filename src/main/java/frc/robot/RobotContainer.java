// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToReefCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.TestPathCommands;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    // #region Controllers
    CommandXboxController primaryController = new CommandXboxController(0);
    // #endregion

    // #region Subsystems
    Swerve swerve = new Swerve();
    //VisionSubsystem visionSubsystem = new VisionSubsystem(swerve);
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    SwerveTelemetry swerveTelemetry = new SwerveTelemetry();
    // #endregion

    // #region Commands
    TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, primaryController);
    // #endregion

    // #region Triggers
    Trigger fastSpeedTrigger = primaryController.rightTrigger();
    Trigger slowSpeedTrigger = primaryController.leftTrigger();

    Trigger elevatorUpTrigger = primaryController.povUp();
    Trigger elevatorDownTrigger = primaryController.povDown();
    // #endregion

    public RobotContainer() {
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
        swerve.setDefaultCommand(teleopSwerve);

        DriverStation.getAlliance().ifPresent(alliance -> {
            swerve.setOperatorPerspective(alliance == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg);
        });

        elevatorSubsystem.disable();
        configureBindings();

    }

    private void configureBindings() {
        fastSpeedTrigger.whileTrue(teleopSwerve.fastSpeedCommand());
        slowSpeedTrigger.whileTrue(teleopSwerve.slowSpeedCommand());

        elevatorUpTrigger.whileTrue(elevatorSubsystem.moveUpManualCommand());
        elevatorDownTrigger.whileTrue(elevatorSubsystem.moveDownManualCommand());
    }

    public Command getAutonomousCommand() {
        return TestPathCommands.moveForwardTest();
    }
}
