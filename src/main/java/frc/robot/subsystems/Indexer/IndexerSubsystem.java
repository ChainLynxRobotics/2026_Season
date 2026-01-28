package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class IndexerSubsystem extends SubsystemBase {
  private TalonFX indexerMotor;
  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  private VelocityVoltage indexerControl = new VelocityVoltage(kGoalIndexerVelocity);
  private VoltageOut voltageOut = new VoltageOut(0.0);

  private SysIdRoutine indexerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(0).per(Seconds), Volts.of(2), Seconds.of(6)),
          new SysIdRoutine.Mechanism(
              volts -> indexerMotor.setControl(voltageOut.withOutput(volts.in(Volts))),
              log ->
                  log.motor("Indexer Motor")
                      .angularPosition(this.getAngularPosition())
                      .angularVelocity(this.getAngularVelocity())
                      .voltage(this.getVoltage()),
              null));

  public IndexerSubsystem(TalonFX indexerMotor) {
    this.indexerMotor = indexerMotor;
    talonFXConfiguration.Slot0 = kIndexerSlot0Config;
    talonFXConfiguration.MotionMagic = kIndexerMotionMagicConfig;
    indexerMotor.getConfigurator().apply(talonFXConfiguration);
  }

  @Logged
  public double getPosition() {
    return indexerMotor.getPosition().getValueAsDouble();
  }

  public Angle getAngularPosition() {
    return indexerMotor.getPosition().getValue();
  }

  @Logged
  public AngularVelocity getAngularVelocity() {
    return indexerMotor.getVelocity().getValue();
  }

  @Logged
  public double getVelocity() {
    return indexerMotor.getVelocity().getValueAsDouble();
  }

  @Logged
  public Voltage getVoltage() {
    return indexerMotor.getMotorVoltage().getValue();
  }

  public void runSysId() {
    run(() -> indexerSysIdRoutine.dynamic(Direction.kForward))
        .andThen(indexerSysIdRoutine.dynamic(Direction.kReverse))
        .andThen(indexerSysIdRoutine.quasistatic(Direction.kForward))
        .andThen(indexerSysIdRoutine.quasistatic(Direction.kReverse));
  }

  public Command spin() {
    return run(() -> indexerMotor.setControl(indexerControl));
  }
}
