package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
  public static Angle kIntakeLowAngle;
  public static Angle kIntakeHighAngle;

  public static AngularVelocity kGoalIntakeSpinVelocity = RotationsPerSecond.of(0.0);

  public static Slot0Configs intakeHeightSlot0Config =
      new Slot0Configs().withKP(0).withKI(0).withKD(0).withKV(0).withKA(0).withKG(0).withKS(0);

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
