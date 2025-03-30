// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class LightsPattern {
    private LEDPattern pattern;

    public LightsPattern(LEDPattern pattern) {
        this.pattern = pattern;
    }

    public LEDPattern getPattern() {
        return pattern;
    }

    public void setPattern(LEDPattern pattern) {
        this.pattern = pattern;
    }

    public static LightsPattern off = new LightsPattern(LEDPattern.kOff);

    public static class Fade extends LightsPattern {
        private double fadeStart;
        private double fadeDuration;
        private Timer timer = new Timer();

        public Fade(LEDPattern pattern, Time fadeStart, Time fadeDuration) {
            super(pattern);
            this.fadeStart = fadeStart.in(Seconds);
            this.fadeDuration = fadeDuration.in(Seconds);
            timer.start();
        }

        public void resetFade() {
            timer.restart();
        }

        private double getFade() {
            return 1 - (timer.get() - fadeStart)/fadeDuration;
        }

        @Override
        public LEDPattern getPattern() {
            return super.pattern.atBrightness(Value.of(getFade()));
        }
    }
}
