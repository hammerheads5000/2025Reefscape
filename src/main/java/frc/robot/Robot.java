// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    CanBridge.runTCP(); // allows using GrappleHook
    m_robotContainer = new RobotContainer();

    // logging
    SmartDashboard.putData(CommandScheduler.getInstance());
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    Epilogue.bind(this);
    SignalLogger.enableAutoLogging(false);
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void disabledInit() {
    m_robotContainer.elevatorSubsystem.setBrake(true);
    m_robotContainer.disabledLightsCommand.schedule();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    m_robotContainer.configureAlliance();
    m_robotContainer.elevatorSubsystem.setBrake(true);
    m_robotContainer.elevatorSubsystem.setRotations(m_robotContainer.elevatorSubsystem.getRotations());
    m_robotContainer.disabledLightsCommand.cancel();
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
