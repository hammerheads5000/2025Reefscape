// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.LightsConstants.*;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
    /** Creates a new LightsSubsystem. */
    AddressableLED ledStrip;
    AddressableLEDBuffer ledBuffer;

    LEDPattern currentPatten = LEDPattern.solid(Color.kBlack);
    boolean animated = false;

    public LEDStrip(int port, int length) {
        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        ledStrip.setLength(ledBuffer.getLength());

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public void setSolidColor(Color color) {
        currentPatten = LEDPattern.solid(color);
        animated = false;
    }

    public void setSteps(Color color1, Color color2, double proportion) {
        currentPatten = LEDPattern.steps(Map.of(0, color1, proportion, color2));
        animated = false;
    }

    public void setRainbow() {
        currentPatten = RAINBOW;
        animated = true;
    }

    public void setPattern(LEDPattern pattern) {
        currentPatten = pattern;
        animated = false;
    }

    public void update() {
        if (!animated) return;

        currentPatten.atBrightness(BRIGHTNESS).applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
    }
}
