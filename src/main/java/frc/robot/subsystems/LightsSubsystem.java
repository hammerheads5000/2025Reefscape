// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.LightsConstants.*;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private enum OverrideMode {
        Rainbow,
        Solid,
        Breathing,
        Blinking,
        Moving
    };

    private SendableChooser<OverrideMode> overrideModeChooser = new SendableChooser<>();

    /** Creates a new LightsSubsystem. */
    public LightsSubsystem() {
        ledStrip.setBitTiming(HIGH_TIME_0_NS, LOW_TIME_0_NS, HIGH_TIME_1_NS, LOW_TIME_1_NS);
        ledStrip.setSyncTime(SYNC_TIME_US);
        ledStrip.setColorOrder(COLOR_ORDER);
        ledStrip.setLength(buffer.getLength());
        ledStrip.start();

        OVERRIDE_ENTRY.set(false);
        COLOR_ENTRY.set("#0000FF");
        SPEED_ENTRY.set(2);

        overrideModeChooser.setDefaultOption("Rainbow", OverrideMode.Rainbow);
        overrideModeChooser.addOption("Solid", OverrideMode.Solid);
        overrideModeChooser.addOption("Breathing", OverrideMode.Breathing);
        overrideModeChooser.addOption("Blinking", OverrideMode.Blinking);
        overrideModeChooser.addOption("Moving", OverrideMode.Moving);

        SmartDashboard.putData("Override Mode", overrideModeChooser);
    }

    private LEDPattern getOverridePattern() {
        Color color;
        try {
            color = new Color(COLOR_ENTRY.get());
        } catch (IllegalArgumentException e) {
            color = Color.kBlue;
        }

        LEDPattern solid = LEDPattern.solid(color);

        switch (overrideModeChooser.getSelected()) {
            case Rainbow:
                return LEDPattern.rainbow(255, 128).scrollAtRelativeSpeed(Hertz.of(SPEED_ENTRY.get()));
            case Solid:
                return solid;
            case Breathing:
                return solid.breathe(Seconds.of(1.0/SPEED_ENTRY.get()));
            case Blinking:
                return solid.blink(Seconds.of(1.0/SPEED_ENTRY.get()));
            case Moving:
                return LEDPattern.gradient(GradientType.kContinuous, 
                        Color.kBlack, Color.kBlack, color, Color.kBlack, Color.kBlack, color)
                        .scrollAtRelativeSpeed(Hertz.of(SPEED_ENTRY.get()));
            default:
                return RAINBOW;
        }
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
        if (OVERRIDE_ENTRY.get()) {
            getOverridePattern().atBrightness(BRIGHTNESS).applyTo(leftView);
            getOverridePattern().atBrightness(BRIGHTNESS).applyTo(rightView);
        } else {
            currentPatternLeft.atBrightness(BRIGHTNESS).applyTo(leftView);
            currentPatternRight.atBrightness(BRIGHTNESS).applyTo(rightView);
        }
        ledStrip.setData(buffer);
    }
}
