package frc.robot.subsystems.led;

import static frc.robot.subsystems.led.LedConstants.kLEDPort;
import static frc.robot.subsystems.led.LedConstants.kLeds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class LedSubsystem {
  public AddressableLED m_led = new AddressableLED(kLEDPort);
  public AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(kLeds);
  public PowerDistribution pDH = new PowerDistribution(1, ModuleType.kRev);

  public LedSubsystem() {
    m_led.setLength(m_buffer.getLength());
    m_led.setData(m_buffer);
    m_led.start();
    pDH.setSwitchableChannel(true);

    setAllLED(Color.kRed);
  }

  public double getLedPower() {
    return pDH.getCurrent(23);
  }

  public void setAllLED(Color color) {
    LEDPattern alliedColor = LEDPattern.solid(color);
    alliedColor.applyTo(m_buffer);
    m_led.setData(m_buffer);
  }

  public void setRainbow() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    rainbow.applyTo(m_buffer);
    m_led.setData(m_buffer);
  }

  public void setProgressBar(Color color, double time) {
    double startTime = Timer.getFPGATimestamp();

    LEDPattern base = LEDPattern.solid(color);
    if ((Timer.getFPGATimestamp() - startTime) / time <= 1) {
      LEDPattern mask =
          LEDPattern.progressMaskLayer(() -> (Timer.getFPGATimestamp() - startTime) / time);
      LEDPattern progress = base.mask(mask);
      progress.applyTo(m_buffer);
    }
    m_led.setData(m_buffer);
  }
}
