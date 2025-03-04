// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.LightsConstants.*;

import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DisabledLightsCommand extends Command {
    LightsSubsystem lightsSubsystem;
    VisionSubsystem visionSubsystem;

    public DisabledLightsCommand(LightsSubsystem lightsSubsystem, VisionSubsystem visionSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    private double getVisionProportion() {
        Distance lastPoseChange = visionSubsystem.lastPoseChange;
        if (lastPoseChange == Meters.of(-1)) return 0;

        return MAX_VISION_POSE_CHANGE.minus(lastPoseChange).div(MAX_VISION_POSE_CHANGE).magnitude();
    }
    
    private double getVisionFade() {
        return 1 - visionSubsystem.timeSinceHadTarget().minus(VISION_TIMEOUT_START).div(VISION_TIMEOUT_DURATION).magnitude();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotController.getMeasureBatteryVoltage().lte(LOW_BATTERY_VOLTAGE)) {
            lightsSubsystem.setSolidColor(LOW_BATTERY_COLOR);
            return;
        }

        if (!visionSubsystem.camerasConnected()) {
            lightsSubsystem.setSolidColor(NO_VISION_COLOR);
            return;
        }
        lightsSubsystem.setSolidColor(Color.kNavy);
        //lightsSubsystem.setSteps(Color.lerpRGB(HAS_TARGET_COLOR, Color.kBlack, getVisionFade()), Color.kBlack, getVisionProportion());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
