package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.Map;

public class ClimberConstants {
  public static final int kClimberId = -1;
  public static final int kLimitSwitchId = 1;
  public static final double kGearRatio = 22.0;
  public static final Angle kSetpointTolerance = Rotations.of(5);

  public static final Time kDT = Seconds.of(0.02);

  public enum ClimberState {
    TOP,
    BOTTOM;
  }

  public static Map<ClimberState, Angle> climberMap =
      Map.of(
          ClimberState.TOP, Rotations.of(120),
          ClimberState.BOTTOM, Rotations.of(0));

  public static final AngularAcceleration kCruiseAcceleration = RotationsPerSecondPerSecond.of(100);
  public static final AngularVelocity kCruiseVelocity = RotationsPerSecond.of(100);
  public static final double kP = 5;
  public static final double kI = 0;
  public static final double kD = 0;

  public static Slot0Configs slot0Configs = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
}
