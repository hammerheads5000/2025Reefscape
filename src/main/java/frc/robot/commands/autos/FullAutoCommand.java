// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.FieldConstants.L1_RELATIVE_POS;

import java.util.Set;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * Command that executes an auto built from a descriptor string
 */
public class FullAutoCommand extends SequentialCommandGroup {
    Swerve swerve;
    ElevatorSubsystem elevatorSubsystem;
    EndEffectorSubsystem endEffectorSubsystem;

    private Command stationCommand(int station) {
        Command command = ApproachCoralStationCommands.pathfindCommand(station, 0, swerve);
                
        if (Robot.isReal()) {
            command = command.alongWith(elevatorSubsystem.goToIntakePosCommand(false))
                    .andThen(endEffectorSubsystem.intakeCommand());
        }
        return command;
    }

    private Command reefCommand(int side, int relativePos, char level) {
        Command commandToAdd;

        Command elevatorPosCommand;
        Command endEffectorCommand = endEffectorSubsystem.scoreCommand();
        switch (level) {
            case '1':
                elevatorPosCommand = elevatorSubsystem.goToL1Command(false);
                endEffectorCommand = relativePos == 1 ? endEffectorSubsystem.troughLeftCommand()
                        : endEffectorSubsystem.troughRightCommand();
                    
                relativePos *= L1_RELATIVE_POS;
                break;
            case '2':
                elevatorPosCommand = elevatorSubsystem.goToL2Command(false);
                break;
            case '3':
                elevatorPosCommand = elevatorSubsystem.goToL3Command(false);
                break;
            case '4':
                elevatorPosCommand = elevatorSubsystem.goToL4Command(false);
                break;
            default:
                elevatorPosCommand = elevatorSubsystem.goToL4Command(false);
                System.err.println("ERROR: Invalid auto level token " + level);
                break;
        }

        commandToAdd = new ApproachReefCommand(side, relativePos, swerve);
        if (Robot.isReal()) {
            commandToAdd = commandToAdd.alongWith(ApproachReefCommand.waitToDeployElevator(side, relativePos, swerve)
                    .andThen(elevatorPosCommand))
                    .andThen(endEffectorSubsystem.scoreCommand());
        }
        return commandToAdd;
    }

    private Command commandFromToken(String token) {
        Command commandToAdd;

        if (token.charAt(0) == 'S') {
            int station = token.charAt(1) == '0' ? 0 : 1;

            commandToAdd = Commands.defer(() -> stationCommand(station), Set.of(swerve, elevatorSubsystem, endEffectorSubsystem));
        } else {
            Pair<Integer, Integer> sidePosPair;
            if (!LETTER_TO_SIDE_AND_RELATIVE.containsKey(token.charAt(0))) {
                System.err.println("ERROR: Invalid auto branch token " + token);
                return Commands.none();
            }
            sidePosPair = LETTER_TO_SIDE_AND_RELATIVE.get(token.charAt(0));
            
            int side = sidePosPair.getFirst();
            int relativePos = sidePosPair.getSecond();

            commandToAdd = Commands.defer(() -> reefCommand(side, relativePos, token.charAt(1)), Set.of(swerve, elevatorSubsystem, endEffectorSubsystem));
        }

        return commandToAdd;
    }

    /**
     * Create FullAutoCommand
     * 
     * @param descriptorString S0-1 (station), A-L1-4 (branch and level), space
     *                         separated (e.g. "E4 S0 A3 S1 K1") 
     * @param swerve
     */
    public FullAutoCommand(String descriptorString, Swerve swerve, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        this.swerve = swerve;
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;

        String[] tokens = descriptorString.split(" ");

        for (String token : tokens) {
            if (token.length() < 2) {
                System.err.println("ERROR: Invalid auto token length " + token);
                continue;
            }
            addCommands(commandFromToken(token));
        }
    }
}
