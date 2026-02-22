package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

@Logged
public class ShooterConstants {
  public static final Pose2d kShooterLocation =
      new Pose2d(Inches.of(8.5), Inches.of(8.5), new Rotation2d(Degrees.of(90).in(Radians)));
  public static final Pose2d kHubLocation =
      new Pose2d(Inches.of(182.11), Inches.of(158.84), new Rotation2d());

  public static final Distance kShooterHeight = Inches.of(18.442849);

  public static final Distance kFlywheelRadius = Inches.of(2);
  public static final MomentOfInertia kFlywheelMOI = KilogramSquareMeters.of(0.00063);
  public static final DCMotor kFlywheelMotor = DCMotor.getKrakenX60Foc(1);
  public static final int kFlywheelCANId = 25;
  public static final double kFlywheelS = 0.18;
  public static final double kFlywheelA = 0.01;
  public static final double kFlywheelV = 0.06325;
  public static final double kFlywheelP = 0.2;
  public static final double kFlywheelI = 0.555;
  public static final double kFlywheelD = 0.0;
  public static final double kFlywheelGearRatio = 0.5;

  protected static TalonFXConfiguration generateFlywheelConfig(
      double kS, double kA, double kV, double kP, double kI, double kD) {
    var gains =
        new Slot0Configs().withKS(kS).withKA(kA).withKV(kV).withKP(kP).withKI(kI).withKD(kD);
    var config = new TalonFXConfiguration().withSlot0(gains);
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = kFlywheelGearRatio;

    // Current Limits
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    return config;
  }

  protected static TalonFXConfiguration generateHoodConfig(
      double kG,
      double kS,
      double kA,
      double kV,
      double kP,
      double kI,
      double kD,
      double gravOffset) {
    var gains =
        new Slot0Configs()
            .withKG(kG)
            .withKS(kS)
            .withKA(kA)
            .withKV(kV)
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withGravityArmPositionOffset(Degrees.of(gravOffset))
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    var config = new TalonFXConfiguration().withSlot0(gains);
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotionMagic.MotionMagicCruiseVelocity = 1;
    config.MotionMagic.MotionMagicAcceleration = 1;
    // config.MotionMagic.MotionMagicExpo_kV = kV * kHoodGearRatio * 1.5;
    // config.MotionMagic.MotionMagicExpo_kA = kA * kHoodGearRatio * 1.5;
    config.Feedback.SensorToMechanismRatio = kHoodGearRatio;

    // Current Limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 50;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    return config;
  }

  protected static final double kEstimatedFlywheelSpeedToFuelSpeed = 0.3;

  public static final TalonFXConfiguration kFlyWheelConfig =
      generateFlywheelConfig(
          kFlywheelS, kFlywheelA, kFlywheelV, kFlywheelP, kFlywheelI, kFlywheelD);

  public static final int kHoodCANId = 26;
  public static final MomentOfInertia kHoodMOI = KilogramSquareMeters.of(0.023948);
  public static final DCMotor kHoodMotor = DCMotor.getKrakenX44Foc(1);
  public static final int kHoodLimitSwitchId = 0;
  protected static final double kHoodG = 0.3225;
  protected static final double kHoodS = 0.25;
  protected static final double kHoodA = 0.01;
  protected static final double kHoodV = 1.25;
  protected static final double kHoodP = 50;
  protected static final double kHoodI = 10;
  protected static final double kHoodD = 0;
  protected static final double kHoodGearRatio = 34;
  public static final TalonFXConfiguration kHoodConfig =
      generateHoodConfig(kHoodG, kHoodS, kHoodA, kHoodV, kHoodP, kHoodI, kHoodD, -0.15);
}
