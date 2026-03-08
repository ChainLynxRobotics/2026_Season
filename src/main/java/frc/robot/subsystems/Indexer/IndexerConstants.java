package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {
  public static final double kIndexerId = 17;
  public static AngularVelocity kGoalIndexerVelocity = RotationsPerSecond.of(12);
  public static double kT = 0.02;

  public static double kIndexerGearRatio = 12;

  public static double kIndexerP = 1;
  public static double kIndexerI = 0.5;
  public static double kIndexerD = 0;

  public static Slot0Configs kIntakeIndexerSlot0Config =
      new Slot0Configs().withKP(kIndexerP).withKI(kIndexerI).withKD(kIndexerD);
}
