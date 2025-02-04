// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
  /** Creates a new LightsSubsystem. */
  AddressableLED ledStrip;
  AddressableLEDBuffer m_ledBuffer;

  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);//saturation out of 255, brightness out of 255


  public LightsSubsystem() {
    ledStrip = new AddressableLED(0);//idk port define in constants
    m_ledBuffer = new AddressableLEDBuffer(60);

    ledStrip.setLength(m_ledBuffer.getLength());

    ledStrip.setData(m_ledBuffer);
    ledStrip.start();
  }

  public void setSolidColor(Color color){//data type should be .kRed or something
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.applyTo(m_ledBuffer);
    ledStrip.setData(m_ledBuffer);
  }

  public void setSteps(Color color1, Color color2, double percent){
    LEDPattern steps = LEDPattern.steps(Map.of(0, color1, percent, color2));//sets first half to be color1, second half color2

    steps.applyTo(m_ledBuffer);
    ledStrip.setData(m_ledBuffer);
  }

  public void setRainbow(){
    m_rainbow.applyTo(m_ledBuffer);
    ledStrip.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
