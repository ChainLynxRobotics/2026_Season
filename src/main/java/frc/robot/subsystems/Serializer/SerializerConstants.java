package frc.robot.subsystems.Serializer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;

public class SerializerConstants {
  public static final double kSerializerId = 18;
  public static AngularVelocity kGoalSerializerVelocity = RotationsPerSecond.of(10.0);
  public static double kT = 0.02;

  private static double kSerializerP = 1;
  private static double kSerializerI = 3;
  private static double kSerializerD = 1.5;

  public static Slot0Configs kIntakeSerializerSlot0Config =
      new Slot0Configs().withKP(kSerializerP).withKI(kSerializerI).withKD(kSerializerD);
}
