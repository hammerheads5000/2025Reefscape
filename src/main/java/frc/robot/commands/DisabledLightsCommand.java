// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static frc.robot.Constants.LightsConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.LightsPattern;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
@Logged
public class DisabledLightsCommand extends Command {
    LightsSubsystem lightsSubsystem;
    VisionSubsystem visionSubsystem;

    private Timer leftFadeTimer = new Timer();
    private Timer rightFadeTimer = new Timer();

    private LEDPattern leftPattern = HAS_TARGET_PATTERN.mask(LEDPattern.progressMaskLayer(this::getVisionProportionL));
    private LEDPattern rightPattern = HAS_TARGET_PATTERN.mask(LEDPattern.progressMaskLayer(this::getVisionProportionR));

    public DisabledLightsCommand(LightsSubsystem lightsSubsystem, VisionSubsystem visionSubsystem) {
        this.lightsSubsystem = lightsSubsystem;
        this.visionSubsystem = visionSubsystem;
    
        leftFadeTimer.start();
        rightFadeTimer.start();
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

    public double getFadeLeft() {
        return 1 - (leftFadeTimer.get() - FADE_START.in(Seconds)) / FADE_DURATION.in(Seconds);
    }

    public double getFadeRight() {
        return 1 - (rightFadeTimer.get() - FADE_START.in(Seconds)) / FADE_DURATION.in(Seconds);
    }

    private void resetFadeLeft() {
        leftFadeTimer.restart();
    }

    private void resetFadeRight() {
        rightFadeTimer.restart();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    //LightsPattern fklejs = new LightsPattern(LEDPattern.solid(Color.kPurple));

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
            resetFadeLeft();
        }
        if (visionSubsystem.hasTargetFR) {
            resetFadeRight();
        }
        
        if (visionSubsystem.flConnected() && visionSubsystem.hasHadTargetFL) {
            lightsSubsystem.setLeftPattern(leftPattern.atBrightness(Value.of(getFadeLeft())));
        } else if(visionSubsystem.flConnected()) {
            lightsSubsystem.setLeftPattern(HAS_NO_TARGET_PATTERN);
        } else {
            lightsSubsystem.setLeftPattern(NO_VISION_PATTERN);
        }

        if (visionSubsystem.frConnected() && visionSubsystem.hasHadTargetFR) {
            lightsSubsystem.setRightPattern(rightPattern.atBrightness(Value.of(getFadeRight())));
        } else if(visionSubsystem.frConnected()) {
            lightsSubsystem.setRightPattern(HAS_NO_TARGET_PATTERN);
        } else {
            lightsSubsystem.setRightPattern(NO_VISION_PATTERN);
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
