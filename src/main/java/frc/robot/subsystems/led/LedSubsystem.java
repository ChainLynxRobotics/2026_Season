package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.led.LedConstants.kBrownOut;
import static frc.robot.subsystems.led.LedConstants.kLEDPort;
import static frc.robot.subsystems.led.LedConstants.kLedSpacing;
import static frc.robot.subsystems.led.LedConstants.kLeds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

@Logged
public class LedSubsystem {
  public AddressableLED led = new AddressableLED(kLEDPort);
  public AddressableLEDBuffer buffer = new AddressableLEDBuffer(kLeds);
  public PowerDistribution pDH = new PowerDistribution(1, ModuleType.kRev);

  public LedSubsystem() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
    pDH.setSwitchableChannel(true);

    setRainbow();
  }

  public double getLedPower() {
    return pDH.getCurrent(23);
  }

  public void setAllLED(Color color) {
    LEDPattern alliedColor = LEDPattern.solid(color);
    alliedColor.applyTo(buffer);
    led.setData(buffer);
  }

  public void setRainbow() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    final LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
    scrollingRainbow.applyTo(buffer);
    led.setData(buffer);
  }

  public void setProgressBar(Color color, double time) {
    double startTime = Timer.getFPGATimestamp();

    LEDPattern base = LEDPattern.solid(color);
    if ((Timer.getFPGATimestamp() - startTime) / time <= 1) {
      LEDPattern mask =
          LEDPattern.progressMaskLayer(() -> (Timer.getFPGATimestamp() - startTime) / time);
      LEDPattern progress = base.mask(mask);
      progress.applyTo(buffer);
    }
    led.setData(buffer);
  }

  public void periodic() {
    if (pDH.getVoltage() < kBrownOut) {
      pDH.setSwitchableChannel(false);
    }
  }
}