package frc.robot.subsystems.Indexer;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IndexerSubsystem extends SubsystemBase {
  private TalonFX indexerMotor = new TalonFX(0);
  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

  private SysIdRoutine indexerSysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(Voltage.of(0).per(Seconds), Volts.of(2), Seconds.of(6)), null);

  public IndexerSubsystem() {
    talonFXConfiguration.Slot0=kIndexerSlot0Config;
    talonFXConfiguration.MotionMagic=kIndexerMotionMagicConfig;
    indexerMotor.getConfigurator().apply(talonFXConfiguration);
  }

  @Logged
  public double getIndexerPosition() {
    return indexerMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getIndexerVelocity() {
    return indexerMotor.getVelocity().getValueAsDouble();
  }

  public Command spin() {
    return run(()->indexerMotor.set(0));
  }
}
