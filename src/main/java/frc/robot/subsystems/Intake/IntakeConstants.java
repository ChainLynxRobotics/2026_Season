package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
  public static Angle kIntakeLowAngle;
  public static Angle kIntakeHighAngle;
  public static AngularVelocity kGoalIntakeSpinVelocity = RotationsPerSecond.of(0);
  public static double kSpinGearRatio = 1 / 3;
  public static double kHeightGearRatio = 1 / 50;
  public static double kT = 0.02;

  private static double kHeightP;
  private static double kHeightI;
  private static double kHeightD;
  private static double kHeightV;
  private static double kHeightA;
  private static double kHeightG;
  private static double kHeightS;

  public static Slot0Configs kIntakeHeightSlot0Config =
      new Slot0Configs()
          .withKP(kHeightP)
          .withKI(kHeightI)
          .withKD(kHeightD)
          .withKV(kHeightV)
          .withKA(kHeightA)
          .withKG(kHeightG)
          .withKS(kHeightS);

  private static double kSpinP;
  private static double kSpinI;
  private static double kSpinD;
  private static double kSpinV;
  private static double kSpinA;
  private static double kSpinG;
  private static double kSpinS;

  public static Slot0Configs kIntakeSpinSlot0Config =
      new Slot0Configs()
          .withKP(kSpinP)
          .withKI(kSpinI)
          .withKD(kSpinD)
          .withKV(kSpinV)
          .withKA(kSpinA)
          .withKG(kSpinG)
          .withKS(kSpinS);

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
