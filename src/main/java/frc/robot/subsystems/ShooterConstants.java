package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

public class ShooterConstants {

  public static final Distance kFlywheelRadius = Inches.of(2);
  public static final MomentOfInertia kFlywheelMOI = KilogramSquareMeters.of(0.0009885367);
  public static final int kFlywheelCANId = 25;
  private static final double kFlywheelS = 0;
  private static final double kFlywheelA = 0.0095487;
  private static final double kFlywheelV = 0.12371;
  private static final double kFlywheelP = 0.095152;
  private static final double kFlywheelI = 0;
  private static final double kFlywheelD = 0;
  public static final double kFlywheelGearRatio = 1;
  private static final Slot0Configs kFlywheelSlot0Configs =
      new Slot0Configs()
          .withKS(kFlywheelS)
          .withKA(kFlywheelA)
          .withKV(kFlywheelV)
          .withKP(kFlywheelP)
          .withKI(kFlywheelI)
          .withKD(kFlywheelD);

  private static TalonFXConfiguration generateFlywheelConfig() {
    var config = new TalonFXConfiguration().withSlot0(kFlywheelSlot0Configs);
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotionMagic.MotionMagicAcceleration = 500;
    config.MotionMagic.MotionMagicCruiseVelocity = 250;
    config.MotionMagic.MotionMagicJerk = 75;
    return config;
  }

  public static final TalonFXConfiguration kFlyWheelConfig = generateFlywheelConfig();

  public static final int kHoodCANId = 26;
  public static final int kHoodLimitSwitchId = 0;
  private static final double kHoodS = 0;
  private static final double kHoodA = 0;
  private static final double kHoodV = 0;
  private static final double kHoodP = 0;
  private static final double kHoodI = 0;
  private static final double kHoodD = 0;
  public static final double kHoodGearRatio = 0;
  private static final Slot0Configs kHoodSlot0Configs =
      new Slot0Configs()
          .withKS(kHoodS)
          .withKA(kHoodA)
          .withKV(kHoodV)
          .withKP(kHoodP)
          .withKI(kHoodI)
          .withKD(kHoodD);
  public static final TalonFXConfiguration kHoodConfig =
      new TalonFXConfiguration().withSlot0(kHoodSlot0Configs);
}
