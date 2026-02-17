package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

@Logged
public class ShooterConstants {
  public static final Pose3d kShooterLocation = new Pose3d();
  public static final Pose3d kHubLocation =
      new Pose3d(Inches.of(182.11), Inches.of(158.84), Inches.of(72), new Rotation3d());

  public static final Distance kFlywheelRadius = Inches.of(2);
  public static final MomentOfInertia kFlywheelMOI = KilogramSquareMeters.of(0.00063);
  public static final DCMotor kFlywheelMotor = DCMotor.getKrakenX60Foc(1);
  public static final int kFlywheelCANId = 25;
  public static final double kFlywheelS = 0.52539;
  public static final double kFlywheelA = 0.015138;
  public static final double kFlywheelV = 0.0675;
  public static final double kFlywheelP = 0.1;
  public static final double kFlywheelI = 0.1;
  public static final double kFlywheelD = 0.01;
  public static final double kFlywheelGearRatio = 0.5;
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
    config.Feedback.SensorToMechanismRatio = kFlywheelGearRatio;

    // Current Limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    return config;
  }

  private static TalonFXConfiguration generateHoodConfig() {
    var config = new TalonFXConfiguration().withSlot0(kHoodSlot0Configs);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotionMagic.MotionMagicAcceleration = 50;
    config.MotionMagic.MotionMagicCruiseVelocity = 25;
    config.MotionMagic.MotionMagicJerk = 75;
    config.Feedback.SensorToMechanismRatio = kHoodGearRatio;

    // Current Limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 50;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    return config;
  }

  protected static final double kEstimatedFlywheelSpeedToFuelSpeed = 0.3;

  public static final TalonFXConfiguration kFlyWheelConfig = generateFlywheelConfig();

  public static final int kHoodCANId = 26;
  public static final MomentOfInertia kHoodMOI = KilogramSquareMeters.of(0.023948);
  public static final DCMotor kHoodMotor = DCMotor.getKrakenX44Foc(1);
  public static final int kHoodLimitSwitchId = 0;
  public static final double kHoodG = 0.3;
  public static final double kHoodS = 0.8;
  public static final double kHoodA = 0.01;
  public static final double kHoodV = 1.25;
  public static final double kHoodP = 50;
  public static final double kHoodI = 0.5;
  public static final double kHoodD = 0;
  public static final double kHoodGearRatio = 34;
  private static final Slot0Configs kHoodSlot0Configs =
      new Slot0Configs()
          .withKG(kHoodG)
          .withKS(kHoodS)
          .withKA(kHoodA)
          .withKV(kHoodV)
          .withKP(kHoodP)
          .withKI(kHoodI)
          .withKD(kHoodD)
          .withGravityType(GravityTypeValue.Arm_Cosine)
          .withGravityArmPositionOffset(Degrees.of(-20));
  public static final TalonFXConfiguration kHoodConfig = generateHoodConfig();
}
