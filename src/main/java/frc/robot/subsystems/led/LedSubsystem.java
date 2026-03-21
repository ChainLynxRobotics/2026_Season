package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static frc.robot.subsystems.led.LedConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class LedSubsystem extends SubsystemBase {
  public AddressableLED led = new AddressableLED(kLEDPort);
  public AddressableLEDBuffer buffer = new AddressableLEDBuffer(kLeds);
  public PowerDistribution pDH = new PowerDistribution(1, ModuleType.kRev);

  public LedSubsystem() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
    pDH.setSwitchableChannel(true);

    LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern m_scrollingRainbow =
        m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), kLedSpacing);

    setDefaultCommand(runPattern(m_scrollingRainbow).withName("Rainbow"));
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));
  }

  @Override
  public void periodic() {
    if (pDH.getVoltage() < kBrownOut) {
      pDH.setSwitchableChannel(false);
    }
    ;

    led.setData(buffer);
  }

  public double getLedPower() {
    return pDH.getCurrent(23);
  }
}
