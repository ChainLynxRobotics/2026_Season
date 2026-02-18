package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {
  public static AngularVelocity kGoalIndexerVelocity = RotationsPerSecond.of(10.0);
  public static double kT = 0.02;

  private static double kIndexerP = 1;
  private static double kIndexerI = 3;
  private static double kIndexerD = 1.5;

  public static Slot0Configs kIntakeIndexerSlot0Config =
      new Slot0Configs().withKP(kIndexerP).withKI(kIndexerI).withKD(kIndexerD);
}
