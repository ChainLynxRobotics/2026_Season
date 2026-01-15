package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import java.util.Map;

public class ClimberConstants {
  public static final int kClimberId = -1;
  public static final int kGearRatio = -1;
  public static final Angle kSetpointTolerance = Rotations.of(-1);

  public enum ClimberState {
    TOP,
    MIDDLE,
    BOTTOM;
  }

  public static Map<ClimberState, Angle> climberMap =
      Map.of(
          ClimberState.TOP, Rotations.of(0 / kGearRatio),
          ClimberState.MIDDLE, Rotations.of(0 / kGearRatio),
          ClimberState.BOTTOM, Rotations.of(0 / kGearRatio));

  public static final AngularAcceleration kCruiseAcceleration = RotationsPerSecondPerSecond.of(-1);
  public static final AngularVelocity kCruiseVelocity = RotationsPerSecond.of(-1);
  public static final Velocity<AngularAccelerationUnit> kMaxJerk =
      RotationsPerSecondPerSecond.of(-1).per(Second);
  public static final double kP = -1;
  public static final double kI = -1;
  public static final double kD = -1;

  public static Slot0Configs slot0Configs = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
}
