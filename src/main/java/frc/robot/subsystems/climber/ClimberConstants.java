package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import java.util.Map;

public class ClimberConstants {
  public static final int kClimberId = 14;
  public static final int kLimitSwitchId = 8;
  public static final double kGearRatio = 23.0;
  public static final Distance kSpoolRadius = Inches.of(0.5);
  public static final Distance kSetpointTolerance = Inches.of(0.2);
  public static final Distance kMaxHeight = Inches.of(8.5);
  public static final Time kDT = Seconds.of(0.02);

  public enum ClimberState {
    TOP,
    BOTTOM;
  }

  public static Map<ClimberState, Distance> climberMap =
      Map.of(
          ClimberState.TOP, Inches.zero(), ClimberState.BOTTOM, kMaxHeight.minus(Inches.of(0.5)));

  public static final double kP = 5;
  public static final double kI = 3;
  public static final double kD = 0.5;
  // meters to rotations
  public static final Per<DistanceUnit, AngleUnit> kPositionConversionFactor =
      kSpoolRadius.times(Math.PI * 2).div(Rotations.one());
  public static final Per<AngleUnit, DistanceUnit> kAngleConversionFactor =
      Rotations.one().div(kSpoolRadius.times(Math.PI * 2));

  public static Slot0Configs slot0Configs = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);

  public static Distance convertToHeight(Angle angle) {
    return (Distance) kPositionConversionFactor.timesDivisor(angle);
  }

  public static Angle convertToAngle(Distance distance) {
    return (Angle) kAngleConversionFactor.timesDivisor(distance);
  }
}
