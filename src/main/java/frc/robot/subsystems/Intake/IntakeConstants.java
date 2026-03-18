package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
  public static final double kIntakeSpinId = 15;
  public static final double kIntakeHeightId = 16;
  public static Angle kIntakeHighAngle = Degree.of(0);
  public static Angle kIntakeLowAngle = Degree.of(124);
  public static AngularVelocity kGoalIntakeSpinVelocity = RotationsPerSecond.of(10);
  public static double kInputToOutputSpinGearRatio = 3;
  public static double kInputToOutputHeightGearRatio = 37.5;
  public static double kT = 0.02;
  public static double kIntakeLengthFromPivot = 0.3347841158;

  public static AngularVelocity kIntakeHeightMaxVelocity = RotationsPerSecond.of(0.123);
  public static AngularAcceleration kIntakeHeightMaxAcceleration =
      RotationsPerSecondPerSecond.of(0.5);

  public static MotionMagicConfigs kIntakeHeightMotionMagic =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(kIntakeHeightMaxVelocity)
          .withMotionMagicAcceleration(kIntakeHeightMaxAcceleration);

  public static double kHeightP = 8;
  public static double kHeightI = 1;
  public static double kHeightD = 0;
  public static double kHeightV = 0.1;
  public static double kHeightA = 0;
  public static double kHeightG = 0;
  public static double kHeightS = 0.75;

  public static Slot0Configs kIntakeHeightSlot0Config =
      new Slot0Configs()
          .withKP(kHeightP)
          .withKI(kHeightI)
          .withKD(kHeightD)
          .withKV(kHeightV)
          .withKA(kHeightA)
          .withKG(kHeightG)
          .withKS(kHeightS);

  // Joe: The method params (kG, kS, etc.) aren't used below — just the static constants. Is that
  // intentional?
  public static TalonFXConfiguration generateHeightConfig(
      double kG,
      double kS,
      double kA,
      double kV,
      double kP,
      double kI,
      double kD,
      double gravOffset) {
    var gains =
        new Slot0Configs()
            .withKP(kHeightP)
            .withKI(kHeightI)
            .withKD(kHeightD)
            .withKV(kHeightV)
            .withKA(kHeightA)
            .withKG(kHeightG)
            .withKS(kHeightS)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(Degrees.of(gravOffset));
    var config = new TalonFXConfiguration().withSlot0(gains);
    // Joe: This sets Coast, but the constructor in IntakeSubsystem sets Brake — could the arm
    // backdrive under gravity after a tunable change?
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotionMagic.MotionMagicCruiseVelocity = kIntakeHeightMaxVelocity.in(RotationsPerSecond);
    config.MotionMagic.MotionMagicAcceleration =
        kIntakeHeightMaxAcceleration.in(RotationsPerSecondPerSecond);
    config.Feedback.SensorToMechanismRatio = kInputToOutputHeightGearRatio;

    config.CurrentLimits.StatorCurrentLimit = 25;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Joe: The constructor doesn't set an inversion but this does — what happens to the arm if the
    // motor direction flips while MotionMagic is holding position?
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    return config;
  }

  public static final Voltage kSpinVoltage = Volts.of(7);

  public static double kSpinP = 1;
  // Joe: I-gain of 3.0 seems pretty high — what happens to the integral if a game piece stalls
  // the roller?
  public static double kSpinI = 3;
  public static double kSpinD = 1.5;

  public static Slot0Configs kIntakeSpinSlot0Config =
      new Slot0Configs().withKP(kSpinP).withKI(kSpinI).withKD(kSpinD);

  public enum IntakeHeightState {
    HIGH(kIntakeHighAngle),
    LOW(kIntakeLowAngle);

    public final Angle angle;

    IntakeHeightState(Angle angle) {
      this.angle = angle;
    }

    public Angle getAngle() {
      return angle;
    }
  }
}
