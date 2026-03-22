package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.led.LedConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
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

    setDefaultCommand(teamColorPattern());
  }

  public Command solidPattern(Color color) {
    LEDPattern pattern = LEDPattern.solid(color);
    return run(() -> pattern.applyTo(buffer)).withName("Solid Color");
  }

  public Command teamColorPattern() {
    LEDPattern pattern = LEDPattern.solid(getAllianceColor());
    return run(() -> pattern.applyTo(buffer)).withName("Alliance Color");
  }

  public Command rainbowScrollPattern() {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), kLedSpacing);
    return run(() -> scrollingRainbow.applyTo(buffer)).withName("Rainbow Scroll");
  }

  public Command progressPattern(double progress, double maximum) {
    LEDPattern pattern = LEDPattern.progressMaskLayer(() -> progress / maximum);
    return run(() -> pattern.applyTo(buffer)).withName("Progress Bar");
  }

  public Command blinkPattern(Color color, Time time) {
    LEDPattern pattern = LEDPattern.solid(color);
    LEDPattern blinkPattern = pattern.blink(time);
    return run(() -> blinkPattern.applyTo(buffer)).withName("Blink Color");
  }

  public Command gradientScrollPattern(Color color1, Color color2, double scrollSpeed) {
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2);
    LEDPattern scrollingRainbow =
        gradient.scrollAtAbsoluteSpeed(MetersPerSecond.of(scrollSpeed), kLedSpacing);
    return run(() -> scrollingRainbow.applyTo(buffer)).withName("Gradient Scroll");
  }

  public Command gradientPattern(Color color1, Color color2) {
    LEDPattern gradient =
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, color1, color2);
    return run(() -> gradient.applyTo(buffer)).withName("Gradient Pattern");
  }

  @Override
  public void periodic() {
    if (pDH.getVoltage() < kBrownOut) {
      pDH.setSwitchableChannel(false);
    }
    ;

    led.setData(buffer);
  }

  public Color getAllianceColor() {
    if (getAlliance().equals(DriverStation.Alliance.Blue)) {
      return Color.kBlue;
    } else {
      return Color.kRed;
    }
  }

  public double getLedPower() {
    return pDH.getCurrent(23);
  }
}
