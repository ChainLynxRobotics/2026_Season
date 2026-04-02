package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {
  public static final double kIndexerId = 17;
  public static AngularVelocity kGoalIndexerVelocity = RotationsPerSecond.of(25);
  public static double kT = 0.02;

  public static double kIndexerGearRatio = 12;

  public static double kIndexerP = 1;
  public static double kIndexerI = 0.0;
  public static double kIndexerD = 0.0;
  public static double kIndexerV = 0.0;
  public static double kIndexerS = 0.55;

  public static double kIndexerUnjamTime = 1.5;
  public static double kIndexerRestartTime = 0.5;

  public static double kIndexerSlowTime = 0.75;
  public static double kIndexerSpeedUpTime = 0.5;
  public static AngularVelocity kIndexerSlowSpeed = RotationsPerSecond.of(-25);

  public static Slot0Configs kIntakeIndexerSlot0Config =
      new Slot0Configs()
          .withKP(kIndexerP)
          .withKI(kIndexerI)
          .withKD(kIndexerD)
          .withKV(kIndexerV)
          .withKS(kIndexerS);
}
