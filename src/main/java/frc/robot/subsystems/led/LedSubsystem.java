package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.subsystems.led.LedConstants.*;

public class LedSubsystem {
  public AddressableLED m_led = new AddressableLED(kLEDPort);
  public AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(kNumberOfPixels);

  public LedSubsystem() {
    m_led.setLength(m_buffer.getLength());
    m_led.setData(m_buffer);
    m_led.start();

    setAllLED(Color.kRed);
  }

  public void setAllLED(Color color) {
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.applyTo(m_buffer);
    m_led.setData(m_buffer);
  }

  public void setRainbow() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    rainbow.applyTo(m_buffer);
    m_led.setData(m_buffer);
  }
}
