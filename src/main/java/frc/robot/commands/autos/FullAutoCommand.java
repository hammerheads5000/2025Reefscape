// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.FieldConstants.L1_RELATIVE_POS;
import static frc.robot.Constants.LightsConstants.ALIGNED_COLOR;
import static frc.robot.Constants.LightsConstants.ALIGNMENT_COLOR;
import static frc.robot.Constants.LightsConstants.IDLE_PATTERN;
import static frc.robot.Constants.LightsConstants.INTAKE_COLOR;

import java.util.Set;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.Swerve;

/**
 * Command that executes an auto built from a descriptor string
 */
public class FullAutoCommand extends SequentialCommandGroup {
    Swerve swerve;
    ElevatorSubsystem elevatorSubsystem;
    EndEffectorSubsystem endEffectorSubsystem;
    LightsSubsystem lightsSubsystem;

    private Command getStationCommand(int station, int relativePos) {
        Command command = ApproachCoralStationCommands.pathfindCommand(station, relativePos, swerve, lightsSubsystem);
                
        if (Robot.isReal()) {
            command = command.alongWith(elevatorSubsystem.goToIntakePosCommand(false))
                    .alongWith(new ScheduleCommand(endEffectorSubsystem.intakeCommand()))
                    .andThen(lightsSubsystem.setSolidColorCommand(INTAKE_COLOR))
                    .andThen(Commands.waitUntil(() -> !endEffectorSubsystem.getBackLidar()))
                    .andThen(lightsSubsystem.setPatternCommand(IDLE_PATTERN));
        }
        return command;
    }

    private Command getStationCommand(int station) {
        return getStationCommand(station, 0);
    }

    private Command getReefCommand(int side, int relativePos, char level) {
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

        elevatorPosCommand = elevatorPosCommand.beforeStarting(() -> {
            lightsSubsystem.setSegmentColor("Top Left", ALIGNMENT_COLOR);
            lightsSubsystem.setSegmentColor("Top Right", ALIGNMENT_COLOR);
            lightsSubsystem.setSegmentColor("Top Back Left", ALIGNMENT_COLOR);
            lightsSubsystem.setSegmentColor("Top Back Right", ALIGNMENT_COLOR);
        }, lightsSubsystem).andThen(() -> {
            lightsSubsystem.setSegmentColor("Top Left", ALIGNED_COLOR);
            lightsSubsystem.setSegmentColor("Top Right", ALIGNED_COLOR);
            lightsSubsystem.setSegmentColor("Top Back Left", ALIGNED_COLOR);
            lightsSubsystem.setSegmentColor("Top Back Right", ALIGNED_COLOR);            
        }, lightsSubsystem);

        commandToAdd = new ApproachReefCommand(side, relativePos, swerve, lightsSubsystem);
        if (Robot.isReal()) {
            commandToAdd = commandToAdd.alongWith(ApproachReefCommand.waitToDeployElevator(side, relativePos, swerve)
                    .andThen(Commands.waitUntil(() -> !endEffectorSubsystem.getFrontLidar()))
                    .andThen(elevatorPosCommand))
                    .andThen(endEffectorCommand.asProxy())
                    .andThen(elevatorSubsystem.goToIntakePosCommand(true));
        }
        return commandToAdd;
    }

    private Command commandFromToken(String token) {
        Command commandToAdd;

        if (token.charAt(0) == 'S') {
            int station = token.charAt(1) == '0' ? 0 : 1;

            Command stationCommand;

            if (token.length() == 3) {
                int relativePos;
                switch (token.charAt(2)) {
                    case 'L':
                        relativePos = 1;
                        break;
                    case 'C':
                        relativePos = 0;
                        break;
                    case 'R':
                        relativePos = -1;
                        break;
                    default:
                        relativePos = 0;
                        System.err.println("ERROR: Invalid auto station token: " + token);
                        break;
                }

                stationCommand = getStationCommand(station, relativePos);
            } else {
                stationCommand = getStationCommand(station);
            }

            commandToAdd = Commands.defer(() -> stationCommand, Set.of(swerve, elevatorSubsystem));
        } else {
            Pair<Integer, Integer> sidePosPair;
            if (!LETTER_TO_SIDE_AND_RELATIVE.containsKey(token.charAt(0))) {
                System.err.println("ERROR: Invalid auto branch token " + token);
                return Commands.none();
            }
            sidePosPair = LETTER_TO_SIDE_AND_RELATIVE.get(token.charAt(0));
            
            int side = sidePosPair.getFirst();
            int relativePos = sidePosPair.getSecond();

            commandToAdd = Commands.defer(() -> getReefCommand(side, relativePos, token.charAt(1)), Set.of(swerve, elevatorSubsystem));
        }

        return commandToAdd;
    }

    /**
     * Create FullAutoCommand
     * 
     * @param descriptorString S0-1[L,C,R] (station and optional relative position), A-L1-4 (branch and level), space
     *                         separated (e.g. "E4 S0C A3 S1 K1") 
     * @param swerve
     */
    public FullAutoCommand(String descriptorString, Swerve swerve, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, LightsSubsystem lightsSubsystem) {
        this.swerve = swerve;
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.lightsSubsystem = lightsSubsystem;

        String[] tokens = descriptorString.split(" ");

        for (String token : tokens) {
            if (token.length() < 2) {
                System.err.println("ERROR: Invalid auto token length " + token);
                continue;
            }
            addCommands(Commands.defer(() -> commandFromToken(token), Set.of(swerve, elevatorSubsystem)));
        }
    }
}
