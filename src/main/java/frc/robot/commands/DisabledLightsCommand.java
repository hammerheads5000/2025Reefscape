// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LightsConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LightsPattern;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
@Logged
public class DisabledLightsCommand extends Command {
    LightsSubsystem lightsSubsystem;
    VisionSubsystem visionSubsystem;

    private boolean previousHasTargetFL = false;
    private boolean previousHasTargetFR = false;

    private LEDPattern leftSteps = LEDPattern.solid(HAS_TARGET_COLOR)
            .mask(LEDPattern.progressMaskLayer(this::getVisionProportionL))
            .overlayOn(LEDPattern.solid(HAS_NO_TARGET_COLOR));
    private LEDPattern rightSteps = LEDPattern.solid(HAS_TARGET_COLOR)
            .mask(LEDPattern.progressMaskLayer(this::getVisionProportionR))
            .overlayOn(LEDPattern.solid(HAS_NO_TARGET_COLOR));

    private LightsPattern.Fade leftPattern = new LightsPattern.Fade(leftSteps, FADE_START, FADE_DURATION);

    private LightsPattern.Fade rightPattern = new LightsPattern.Fade(rightSteps, FADE_START, FADE_DURATION);

    public DisabledLightsCommand(LightsSubsystem lightsSubsystem, VisionSubsystem visionSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    public double getVisionProportionL() {
        if (!visionSubsystem.hasHadTargetFL)
            return 0;

        Distance distance = visionSubsystem.getDistanceToEstimatedFL();
        return MAX_VISION_DISTANCE.minus(distance).div(MAX_VISION_DISTANCE).magnitude();
    }

    public double getVisionProportionR() {
        if (!visionSubsystem.hasHadTargetFR)
            return 0;

        Distance distance = visionSubsystem.getDistanceToEstimatedFR();
        return MAX_VISION_DISTANCE.minus(distance).div(MAX_VISION_DISTANCE).magnitude();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // lightsSubsystem.setRainbow();
        // if (!DriverStation.isFMSAttached()) {
        //     return;
        // }

        // if (RobotController.getMeasureBatteryVoltage().lte(LOW_BATTERY_VOLTAGE)) {
        //     lightsSubsystem.setSolidColor(LOW_BATTERY_COLOR);
        //     return;
        // }

        if (visionSubsystem.hasTargetFL) {
            leftPattern.resetFade();
        }
        if (visionSubsystem.hasTargetFR) {
            rightPattern.resetFade();
        }

        previousHasTargetFL = visionSubsystem.hasTargetFL;
        previousHasTargetFR = visionSubsystem.hasTargetFR;
        
        if (visionSubsystem.flConnected()) {
            lightsSubsystem.setLeftPattern(leftPattern);
        } else {
            lightsSubsystem.setColorLeft(NO_VISION_COLOR);
        }

        if (visionSubsystem.frConnected()) {
            lightsSubsystem.setRightPattern(rightPattern);
        } else {
            lightsSubsystem.setColorRight(NO_VISION_COLOR);
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
