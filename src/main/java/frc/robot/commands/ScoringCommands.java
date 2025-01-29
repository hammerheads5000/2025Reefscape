// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/** Add your docs here. */
public class ScoringCommands {
    public static Command scoreL1LeftCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        return Commands.sequence(
            elevatorSubsystem.goToHeightCommand(false, L1_HEIGHT),
            endEffectorSubsystem.troughLeftCommand()
        );
    }

    public static Command scoreL1RightCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        return Commands.sequence(
            elevatorSubsystem.goToHeightCommand(false, L1_HEIGHT),
            endEffectorSubsystem.troughRightCommand()
        );
    }

    public static Command scoreL2Command(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        return Commands.sequence(
            elevatorSubsystem.goToHeightCommand(false, L2_HEIGHT),
            endEffectorSubsystem.scoreCommand()
        );
    }

    public static Command scoreL3Command(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        return Commands.sequence(
            elevatorSubsystem.goToHeightCommand(false, L3_HEIGHT),
            endEffectorSubsystem.scoreCommand()
        );
    }

    public static Command scoreL4Command(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        return Commands.sequence(
            elevatorSubsystem.goToHeightCommand(false, L4_HEIGHT),
            endEffectorSubsystem.scoreCommand()
        );
    }
}
