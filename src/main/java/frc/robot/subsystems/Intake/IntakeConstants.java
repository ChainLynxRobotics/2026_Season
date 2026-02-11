package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
  public static Angle kIntakeLowAngle = Degree.of(0);
  public static Angle kIntakeHighAngle = Degree.of(124);
  public static AngularVelocity kGoalIntakeSpinVelocity = RotationsPerSecond.of(10);
  public static double kInputToOutputSpinGearRatio = 3;
  public static double kInputToOutputHeightGearRatio = 60;
  public static double kT = 0.02;
  public static double kIntakeLengthFromPivot = 0.3347841158;

  public static AngularVelocity kIntakeHeightMaxVelocity = RotationsPerSecond.of(100);
  public static AngularAcceleration kIntakeHeightMaxAcceleration =
      RotationsPerSecondPerSecond.of(100);

  public static MotionMagicConfigs kIntakeHeightMotionMagic =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(kIntakeHeightMaxVelocity)
          .withMotionMagicAcceleration(kIntakeHeightMaxAcceleration);

  private static double kHeightP = 0;
  private static double kHeightI = 0;
  private static double kHeightD = 0;
  public static double kHeightV = 7.47;
  public static double kHeightA = 0.02;
  private static double kHeightG = 0.09;
  private static double kHeightS = 0;

  public static Slot0Configs kIntakeHeightSlot0Config =
      new Slot0Configs()
          .withKP(kHeightP)
          .withKI(kHeightI)
          .withKD(kHeightD)
          .withKV(kHeightV)
          .withKA(kHeightA)
          .withKG(kHeightG)
          .withKS(kHeightS);

  private static double kSpinP = 2;
  private static double kSpinI = 0.25;
  private static double kSpinD = 0;
  private static double kSpinV = 0;
  private static double kSpinA = 0;
  private static double kSpinG = 0;
  private static double kSpinS = 0;

  public static Slot0Configs kIntakeSpinSlot0Config =
      new Slot0Configs().withKP(kSpinP).withKI(kSpinI).withKD(kSpinD)
      // .withKV(kSpinV)
      // .withKA(kSpinA)
      // .withKG(kSpinG)
      // .withKS(kSpinS)
      ;

  public enum IntakeHeightState {
    HIGH(kIntakeHighAngle.times(kInputToOutputHeightGearRatio)),
    LOW(kIntakeLowAngle.times(kInputToOutputHeightGearRatio));

    public final Angle angle;

    IntakeHeightState(Angle angle) {
      this.angle = angle;
    }

    public Angle getAngle() {
      return angle;
    }
  }
}
