package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;

public class IntakeConstants {
  public static Angle kIntakeLowAngle = Degrees.of(124);
  public static Angle kIntakeHighAngle = Rotations.of(0);
  public static AngularVelocity kGoalIntakeSpinVelocity = RotationsPerSecond.of(10);
  public static double kOutputToInputSpinGearRatio = 1.0 / 3;
  public static double kOutputToInputHeightGearRatio = 1.0 / 60;
  public static double kT = 0.02;

  public static double kIntakeHeightMaxVelocity = 100;
  public static double kIntakeHeightMaxAcceleration = 100;

  public static MotionMagicConfigs kIntakeHeightMotionMagic =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(kIntakeHeightMaxVelocity)
          .withMotionMagicAcceleration(kIntakeHeightMaxAcceleration);

  private static double kHeightP = 5;
  private static double kHeightI = 0;
  private static double kHeightD = 0;
  //   private static double kHeightV = 0;
  //   private static double kHeightA = 0;
  //   private static double kHeightG = 0;
  //   private static double kHeightS = 0;

  public static Slot0Configs kIntakeHeightSlot0Config =
      new Slot0Configs().withKP(kHeightP).withKI(kHeightI).withKD(kHeightD)
      //   .withKV(kHeightV)
      //   .withKA(kHeightA)
      //   .withKG(kHeightG)
      //   .withKS(kHeightS)
      ;

  private static double kSpinP = 0.65;
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
