package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.led.LedConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Map;

@Logged
public class LedSubsystem extends SubsystemBase {
  public AddressableLED led = new AddressableLED(kLEDPort);
  public AddressableLEDBuffer buffer = new AddressableLEDBuffer(kLeds);
  public PowerDistribution pDH = new PowerDistribution(1, ModuleType.kRev);
  public boolean didYouWinAuto;

  public LedSubsystem() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
    pDH.setSwitchableChannel(true);

    setDefaultCommand(solidPattern(Color.kGreen));
  }

  public Command solidPattern(Color color) {
    Color formattedColor = formatColorForRBG(color);
    LEDPattern pattern = LEDPattern.solid(formattedColor);
    return run(() -> pattern.applyTo(buffer)).withName("Solid Color");
  }

  public Command teamColorPattern() {
    LEDPattern pattern = LEDPattern.solid(formatColorForRBG(getAllianceColor()));
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

  public Command blinkPattern(Color color, double time) {
    Color formattedColor = formatColorForRBG(color);
    LEDPattern pattern = LEDPattern.solid(formattedColor);
    LEDPattern blinkPattern = pattern.blink(Milliseconds.of(time));
    return run(() -> blinkPattern.applyTo(buffer)).withName("Blink Color");
  }

  public Command gradientScrollPattern(Color color1, Color color2, double scrollSpeed) {
    Color formattedColor1 = formatColorForRBG(color1);
    Color formattedColor2 = formatColorForRBG(color2);
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, formattedColor1, formattedColor2);
    LEDPattern scroll =
        gradient.scrollAtAbsoluteSpeed(MetersPerSecond.of(scrollSpeed), kLedSpacing);
    return run(() -> scroll.applyTo(buffer)).withName("Gradient Scroll");
  }

  public Command gradientPattern(Color color1, Color color2) {
    Color formattedColor1 = formatColorForRBG(color1);
    Color formattedColor2 = formatColorForRBG(color2);
    LEDPattern gradient =
        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, formattedColor1, formattedColor2);
    return run(() -> gradient.applyTo(buffer)).withName("Gradient Pattern");
  }

  public Command maskScrollPattern(
      Map<Double, Color> steps, LEDPattern base, double speed, boolean reversed) {
    LEDPattern mask = LEDPattern.steps(steps).scrollAtRelativeSpeed(Percent.per(Second).of(speed));

    LEDPattern pattern;

    if (reversed) {
      pattern = base.mask(mask).reversed();
    } else {
      pattern = base.mask(mask);
    }

    return run(() -> pattern.applyTo(buffer)).withName("Mask Scroll Pattern");
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

  public void calculateShifts() {
    if (DriverStation.getGameSpecificMessage().equals("B")
        && getAlliance().equals(DriverStation.Alliance.Blue)) {
      didYouWinAuto = true;
    } else if (DriverStation.getGameSpecificMessage().equals("R")
        && getAlliance().equals(DriverStation.Alliance.Red)) {
      didYouWinAuto = true;
    } else {
      didYouWinAuto = false;
    }
  }  

  public Color formatColorForRBG(Color color) {
    return new Color(color.red, color.blue, color.green);
  }

  public double getLedPower() {
    return pDH.getCurrent(23);
  }

  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public Command activePhasePattern() {
    return maskScrollPattern(
        Map.of(
            0.0,
            Color.kWhite,
            0.1,
            Color.kBlack,
            0.2,
            Color.kWhite,
            0.3,
            Color.kBlack,
            0.4,
            Color.kWhite,
            0.5,
            Color.kBlack,
            0.6,
            Color.kWhite,
            0.7,
            Color.kBlack,
            0.8,
            Color.kWhite,
            0.9,
            Color.kBlack),
        LEDPattern.solid(formatColorForRBG(getAllianceColor())),
        8,
        false);
  }
  ;

  public Command defendingPhasePattern() {
    return gradientPattern(Color.kGreen, Color.kWhite);
  }
  ;

  public Command autonomousPattern() {
    return gradientScrollPattern(getAllianceColor(), Color.kWhite, 5);
  }
  ;

  public Command hubShiftPattern() {
    return blinkPattern(Color.kPurple, 200);
  }
  ;

  public Command endGamePattern() {
    return gradientScrollPattern(Color.kOrange, Color.kYellow, 8);
  }
  ;

  public Command shootPattern() {
    return maskScrollPattern(
        Map.of(
            0.0,
            Color.kWhite,
            0.1,
            Color.kBlack,
            0.2,
            Color.kWhite,
            0.3,
            Color.kBlack,
            0.4,
            Color.kWhite,
            0.5,
            Color.kBlack,
            0.6,
            Color.kWhite,
            0.7,
            Color.kBlack,
            0.8,
            Color.kWhite,
            0.9,
            Color.kBlack),
        LEDPattern.solid(formatColorForRBG(Color.kWhite)),
        16,
        true);
  }

  public enum RobotState {
    AUTO,
    ACTIVE,
    INACTIVE,
    SHIFTCHANGE,
    ENDGAME,
    DISABLED,
    SHOOTING
  }

  private final CommandXboxController driveController = new CommandXboxController(0);

  public RobotState getRobotState() {
    double time = DriverStation.getMatchTime();

    if ((time > 130 && time < 135)
        || (time > 105 && time < 110)
        || (time > 80 && time < 85)
        || (time > 55 && time < 60)
        || (time > 30 && time < 35)) {
      return RobotState.SHIFTCHANGE;
    }
    if (DriverStation.isAutonomousEnabled()) return RobotState.AUTO;
    if (driveController.getRightTriggerAxis() > 0.5) return RobotState.SHOOTING;

    if (time > -1 && time < 30) return RobotState.ENDGAME;

    if (time > 135) return RobotState.ACTIVE;

    if ((time > 110 && time < 130) || (time > 60 && time < 80)) {
      if (didYouWinAuto) {
        return RobotState.INACTIVE;
      } else {
        return RobotState.ACTIVE;
      }
    } else if ((time > 85 && time < 105) || (time > 35 && time < 55)) {
      if (didYouWinAuto) {
        return RobotState.ACTIVE;
      } else {
        return RobotState.INACTIVE;
      }
    } else {
      return RobotState.DISABLED;
    }
  }
}
