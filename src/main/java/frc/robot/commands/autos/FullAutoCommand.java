// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static frc.robot.Constants.AutoConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullAutoCommand extends SequentialCommandGroup {
    /**
     * Create FullAutoCommand
     * 
     * @param descriptorString S0-1 (station), A-L1-4 (branch and level), space
     *                         separated
     * @param swerve
     */
    public FullAutoCommand(String descriptorString, Swerve swerve, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        String[] tokens = descriptorString.split(" ");

        for (String token : tokens) {
            Command commandToAdd;
            if (token.charAt(0) == 'S') {
                int station = token.charAt(1) == '0' ? 0 : 1;
                SmartDashboard.putBoolean(token, isScheduled());
                commandToAdd = ApproachCoralStationCommands.pathfindCommand(station, 0, swerve)
                    .alongWith(elevatorSubsystem.goToIntakePosCommand(false))
                    .andThen(endEffectorSubsystem.intakeCommand());
            }
            else {
                Pair<Integer, Integer> sidePosPair = LETTER_TO_SIDE_AND_RELATIVE.get(token.charAt(0));
                int side = sidePosPair.getFirst();
                int relativePos = sidePosPair.getSecond();

                Command elevatorPosCommand;
                switch (token.charAt(1)) {
                    case '1':
                        elevatorPosCommand = elevatorSubsystem.goToL1Command(false);
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
                        break;
                }

                commandToAdd = new ApproachReefCommand(side, relativePos, swerve)
                    .alongWith(elevatorPosCommand)
                    .andThen(endEffectorSubsystem.scoreCommand());
            }

            addCommands(commandToAdd);
        }
    }
}
