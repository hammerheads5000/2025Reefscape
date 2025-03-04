// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LightsConstants.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LEDStrip;

public class LightsSubsystem extends SubsystemBase {
    LEDStrip leftStrip = new LEDStrip(PWM_PORT_LEFT, LED_COUNT_LEFT);
    LEDStrip rightStrip = new LEDStrip(PWM_PORT_LEFT, LED_COUNT_RIGHT);

    /** Creates a new LightsSubsystem. */
    public LightsSubsystem() {
    }

    public void setSolidColor(Color color) {
        leftStrip.setSolidColor(color);
        rightStrip.setSolidColor(color);
    }

    public void setSteps(Color color1, Color color2, double proportion) {
        leftStrip.setSteps(color1, color2, proportion);
        rightStrip.setSteps(color1, color2, proportion);
    }

    public void setRainbow() {
        leftStrip.setRainbow();
        rightStrip.setRainbow();
    }

    public void setPattern(LEDPattern pattern) {
        leftStrip.setPattern(pattern);
        rightStrip.setPattern(pattern);
    }

    @Override
    public void periodic() {
        leftStrip.update();
        rightStrip.update();
    }
}
