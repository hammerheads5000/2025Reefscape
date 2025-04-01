// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LightsConstants.*;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
    AddressableLED ledStrip = new AddressableLED(PWM_PORT);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_LEFT + LEDS_RIGHT);

    AddressableLEDBufferView rightView = buffer.createView(0, LEDS_RIGHT);
    AddressableLEDBufferView leftView = buffer.createView(LEDS_RIGHT + 1, LEDS_LEFT + LEDS_RIGHT - 1).reversed();

    private LEDPattern currentPatternLeft = LEDPattern.kOff;
    private LEDPattern currentPatternRight = LEDPattern.kOff;

    /** Creates a new LightsSubsystem. */
    public LightsSubsystem() {
        ledStrip.setBitTiming(HIGH_TIME_0_NS, LOW_TIME_0_NS, HIGH_TIME_1_NS, LOW_TIME_1_NS);
        ledStrip.setSyncTime(SYNC_TIME_US);
        ledStrip.setColorOrder(COLOR_ORDER);
        ledStrip.setLength(buffer.getLength());
        ledStrip.start();
    }

    public void setLeftPattern(LEDPattern pattern) {
        currentPatternLeft = pattern;
    }
    
    public void setRightPattern(LEDPattern pattern) {
        currentPatternRight = pattern;
    }
    
    public void setPatterns(LEDPattern patternLeft, LEDPattern patternRight) {
        setLeftPattern(patternLeft);
        setRightPattern(patternRight);
    }
    
    public void setPattern(LEDPattern pattern) {
        setPatterns(pattern, pattern);
    }

    public void setSolidColor(Color color) {
        setColorLeft(color);
        setColorRight(color);
    }

    public void setColorLeft(Color color) {
        setLeftPattern(LEDPattern.solid(color).atBrightness(BRIGHTNESS));
    }

    public void setColorRight(Color color) {
        setRightPattern(LEDPattern.solid(color).atBrightness(BRIGHTNESS));
    }

    public void setStepsLeft(Color color1, Color color2, double proportion) {
        setLeftPattern(LEDPattern.steps(Map.of(0, color1, proportion, color2)).atBrightness(BRIGHTNESS));
    }

    public void setStepsRight(Color color1, Color color2, double proportion) {
        setPatterns(currentPatternLeft,
                LEDPattern.steps(Map.of(0, color1, proportion, color2)).atBrightness(BRIGHTNESS));
    }

    public void setSteps(Color color1, Color color2, double proportion) {
        setStepsLeft(color1, color2, proportion);
        setStepsRight(color1, color2, proportion);
    }

    public void setRainbow() {
        setPattern(RAINBOW);
    }

    public Command setSolidColorCommand(Color color) {
        return this.runOnce(() -> this.setSolidColor(color));
    }

    public Command setStepsCommand(Color color1, Color color2, double proportion) {
        return this.runOnce(() -> this.setSteps(color1, color2, proportion));
    }

    public Command setRainbowCommand() {
        return this.runOnce(() -> this.setRainbow());
    }

    public Command setPatternCommand(LEDPattern pattern) {
        return this.runOnce(() -> this.setPattern(pattern));
    }

    @Override
    public void periodic() {
        currentPatternLeft.atBrightness(BRIGHTNESS).applyTo(leftView);
        currentPatternRight.atBrightness(BRIGHTNESS).applyTo(rightView);

        ledStrip.setData(buffer);
    }
}
