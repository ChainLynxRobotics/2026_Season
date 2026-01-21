package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
  public static Angle kIntakeLowAngle;
  public static Angle kIntakeHighAngle;
  private static double kP;
  private static double kI;
  private static double kD;
  private static double kV;
  private static double kA;
  private static double kG;
  private static double kS;
  public static Slot0Configs kIntakeHeightSlot0Config =
      new Slot0Configs()
          .withKP(kP)
          .withKI(kI)
          .withKD(kD)
          .withKV(kV)
          .withKA(kA)
          .withKG(kG)
          .withKS(kS);

  public enum IntakeState {
    HIGH(kIntakeHighAngle),
    LOW(kIntakeLowAngle);

    public final Angle angle;

    IntakeState(Angle angle) {
      this.angle = angle;
    }

    public Angle getAngle() {
      return angle;
    }
  }
}