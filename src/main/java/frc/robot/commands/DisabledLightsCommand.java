// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static frc.robot.Constants.LightsConstants.*;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DisabledLightsCommand extends Command {
    LightsSubsystem lightsSubsystem;
    VisionSubsystem visionSubsystem;

    private boolean previousHasTargetFL = false;
    private boolean previousHasTargetFR = false;

    private Timer fadeLeft = new Timer();
    private Timer fadeRight = new Timer();

    public DisabledLightsCommand(LightsSubsystem lightsSubsystem, VisionSubsystem visionSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    private double getVisionProportionL() {
        if (!visionSubsystem.hasHadTargetFL) return 0;

        Distance distance = visionSubsystem.getDistanceToEstimatedFL();
        return MAX_VISION_DISTANCE.minus(distance).div(MAX_VISION_DISTANCE).magnitude();
    }

    private double getVisionProportionR() {
        if (!visionSubsystem.hasHadTargetFR) return 0;
        
        Distance distance = visionSubsystem.getDistanceToEstimatedFR();
        return MAX_VISION_DISTANCE.minus(distance).div(MAX_VISION_DISTANCE).magnitude();
    }

    public double getFadeLeft() {
        Time fadeTime = Seconds.of(fadeLeft.get());
        return 1 - fadeTime.minus(FADE_START).div(FADE_DURATION).in(Value);
    } 

    public double getFadeRight() {
        Time fadeTime = Seconds.of(fadeRight.get());
        return 1 - fadeTime.minus(FADE_START).div(FADE_DURATION).in(Value);
    } 


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        lightsSubsystem.setRainbow();
        if (!DriverStation.isFMSAttached()) {
            return;
        }

        if (RobotController.getMeasureBatteryVoltage().lte(LOW_BATTERY_VOLTAGE)) {
            lightsSubsystem.setSegmentColor("Top Left", LOW_BATTERY_COLOR);
            lightsSubsystem.setSegmentColor("Top Right", LOW_BATTERY_COLOR);
            return;
        }

        if (visionSubsystem.hasTargetFL && !previousHasTargetFL) {
            fadeLeft.restart();
        }
        if (visionSubsystem.hasTargetFR && !previousHasTargetFR) {
            fadeRight.restart();
        }

        previousHasTargetFL = visionSubsystem.hasTargetFL;
        previousHasTargetFR = visionSubsystem.hasTargetFR;
        
        if (visionSubsystem.flConnected()) {
            LEDPattern pattern = LEDPattern.steps(Map.of(
                    0, Color.lerpRGB(HAS_NO_TARGET_COLOR, HAS_TARGET_COLOR, getFadeLeft()),
                    getVisionProportionL(), HAS_NO_TARGET_COLOR));
            lightsSubsystem.setSegmentPattern("Front Left", pattern);
        } else {
            lightsSubsystem.setSegmentColor("Front Left", NO_VISION_COLOR);
        }

        if (visionSubsystem.frConnected()) {
            LEDPattern pattern = LEDPattern.steps(Map.of(
                    0, Color.lerpRGB(HAS_NO_TARGET_COLOR, HAS_TARGET_COLOR, getFadeRight()), 
                    getVisionProportionR(), HAS_NO_TARGET_COLOR));
            lightsSubsystem.setSegmentPattern("Front Right", pattern);
        } else {
            lightsSubsystem.setSegmentColor("Front Right", NO_VISION_COLOR);
        }
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
