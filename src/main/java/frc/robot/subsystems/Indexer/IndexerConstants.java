package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class IndexerConstants {
  private static double kP;
  private static double kI;
  private static double kD;
  private static double kV;
  private static double kA;
  private static double kG;
  private static double kS;
  private static double kMaxVelocity;
  public static Slot0Configs kIndexerSlot0Config =
      new Slot0Configs()
          .withKP(kP)
          .withKI(kI)
          .withKD(kD)
          .withKV(kV)
          .withKA(kA)
          .withKG(kG)
          .withKS(kS);
  public static MotionMagicConfigs kIndexerMotionMagicConfig =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(kMaxVelocity);
}
