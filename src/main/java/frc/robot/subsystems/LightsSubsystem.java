// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;
import static frc.robot.Constants.LightsConstants.*;

import java.util.Map;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
    // AddressableLED ledStrip = new AddressableLED(PWM_PORT);
    // AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_SEPARATIONS[7]+1);
    
    // AddressableLEDBufferView rightView = buffer.createView(0, LED_SEPARATIONS[3]);
    // AddressableLEDBufferView leftView = buffer.createView(LED_SEPARATIONS[3]+1, LED_SEPARATIONS[7]);
    
    // AddressableLEDBufferView frontRightView = buffer.createView(0, LED_SEPARATIONS[0]);
    // AddressableLEDBufferView topRightView = buffer.createView(LED_SEPARATIONS[0]+1, LED_SEPARATIONS[1]);
    // AddressableLEDBufferView topBackRightView = buffer.createView(LED_SEPARATIONS[1]+1, LED_SEPARATIONS[2]).reversed();
    // AddressableLEDBufferView backRightView = buffer.createView(LED_SEPARATIONS[2]+1, LED_SEPARATIONS[3]).reversed();
    
    // AddressableLEDBufferView frontLeftView = buffer.createView(LED_SEPARATIONS[3]+1, LED_SEPARATIONS[4]);
    // AddressableLEDBufferView topLeftView = buffer.createView(LED_SEPARATIONS[4]+1, LED_SEPARATIONS[5]);
    // AddressableLEDBufferView topBackLeftView = buffer.createView(LED_SEPARATIONS[5]+1, LED_SEPARATIONS[6]).reversed();
    // AddressableLEDBufferView backLeftView = buffer.createView(LED_SEPARATIONS[6]+1, LED_SEPARATIONS[7]).reversed();

    // Map<String, AddressableLEDBufferView> views = Map.of(
    //     "Left", leftView,
    //     "Right", rightView,
    //     "Front Right", frontRightView,
    //     "Top Right", topRightView,
    //     "Top Back Right", topBackRightView,
    //     "Back Right", backRightView,
    //     "Front Left", frontLeftView,
    //     "Top Left", topLeftView,
    //     "Top Back Left", topBackLeftView,
    //     "Back Left", backLeftView
    // );


    private LEDPattern currentPatternLeft = LEDPattern.kOff;
    private LEDPattern currentPatternRight = LEDPattern.kOff;

    private Time lastUpdateLeft = RobotController.getMeasureTime();
    private Time lastUpdateRight = RobotController.getMeasureTime();

    private boolean shouldFade = false;

    /** Creates a new LightsSubsystem. */
    public LightsSubsystem() {
        // ledStrip.setBitTiming(HIGH_TIME_0_NS, LOW_TIME_0_NS, HIGH_TIME_1_NS, LOW_TIME_1_NS);
        // ledStrip.setSyncTime(SYNC_TIME_US);
        // ledStrip.setColorOrder(COLOR_ORDER);
        // ledStrip.setLength(buffer.getLength());
        // ledStrip.start();
    }

    public void resetFadeLeft() {
        lastUpdateLeft = RobotController.getMeasureTime();
    }
    
    public void resetFadeRight() {
        lastUpdateRight = RobotController.getMeasureTime();
    }
    
    public void resetFade() {
        resetFadeLeft();
        resetFadeRight();
    }

    public void setShouldFade(boolean shouldFade) {
        this.shouldFade = shouldFade;
    }

    public void setPatterns(LEDPattern patternLeft, LEDPattern patternRight) {
        currentPatternLeft = patternLeft;
        currentPatternRight = patternRight;
    }

    public void setPattern(LEDPattern pattern) {
        setPatterns(pattern, pattern);
    }

    public void setSolidColor(Color color) {
        setPattern(LEDPattern.solid(color));
    }

    public void setColorLeft(Color color) {
        setPatterns(LEDPattern.solid(color), currentPatternRight);
    }

    public void setColorRight(Color color) {
        setPatterns(currentPatternLeft, LEDPattern.solid(color));
    }

    public void setStepsLeft(Color color1, Color color2, double proportion) {
        setPatterns(LEDPattern.steps(Map.of(0, color1, proportion, color2)), currentPatternRight);
    }

    public void setStepsRight(Color color1, Color color2, double proportion) {
        setPatterns(currentPatternLeft, LEDPattern.steps(Map.of(0, color1, proportion, color2)));
    }

    public void setSteps(Color color1, Color color2, double proportion) {
        setStepsLeft(color1, color2, proportion);
        setStepsRight(color1, color2, proportion);
    }

    public void setRainbow() {
        setPattern(RAINBOW);
    }

    public void setSegmentPattern(String segment, LEDPattern pattern) {
        // AddressableLEDBufferView view = views.get(segment);
        // pattern.applyTo(view);
    }

    public void setSegmentColor(String segment, Color color) {
        setSegmentPattern(segment, LEDPattern.solid(color));
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

    @Logged
    public Dimensionless getFadeLeft() {
        if (!shouldFade) return Value.of(1);

        Time fadeTime = RobotController.getMeasureTime().minus(lastUpdateLeft);
        return Value.of(1).minus(fadeTime.minus(FADE_START).div(FADE_DURATION));
    } 

    @Logged
    public Dimensionless getFadeRight() {
        if (!shouldFade) return Value.of(1);

        Time fadeTime = RobotController.getMeasureTime().minus(lastUpdateRight);
        return Value.of(1).minus(fadeTime.minus(FADE_START).div(FADE_DURATION));
    } 

    @Override
    public void periodic() {
        // currentPatternLeft.atBrightness(BRIGHTNESS.times(getFadeLeft())).applyTo(leftView);
        // currentPatternRight.atBrightness(BRIGHTNESS.times(getFadeRight())).applyTo(rightView);
        
        // ledStrip.setData(buffer);
        
    }
}
