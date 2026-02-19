package frc.robot.subsystems.Indexer;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class IndexerSubsystem extends SubsystemBase {
  private TalonFX indexerMotor;
  private TalonFXConfiguration indexerConfiguration = new TalonFXConfiguration();
  private VoltageOut indexerControl = new VoltageOut(Volts.of(0));

  private DCMotor x44Gearbox = DCMotor.getKrakenX44Foc(1);
  private TalonFXSimState indexerSimState;
  private DCMotorSim indexerSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(x44Gearbox, 0.001, 1), x44Gearbox);

  private VoltageOut voltageOut = new VoltageOut(0.0);

  private SysIdRoutine indexerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(0.5).per(Seconds), Volts.of(2), Seconds.of(6)),
          new SysIdRoutine.Mechanism(
              volts -> indexerMotor.setControl(voltageOut.withOutput(volts.in(Volts))),
              log ->
                  log.motor("Leader")
                      .angularPosition(Rotations.of(this.getIndexerPosition()))
                      .angularVelocity(RotationsPerSecond.of(this.getIndexerVelocity()))
                      .voltage(this.getIndexerVoltage()),
              this));

  public IndexerSubsystem(TalonFX indexerMotor) {
    this.indexerMotor = indexerMotor;

    indexerConfiguration.Slot0 = kIntakeIndexerSlot0Config;
    indexerMotor.getConfigurator().apply(indexerConfiguration);

    indexerSimState = indexerMotor.getSimState();
  }

  public void runSysId() {
    run(() -> indexerSysIdRoutine.dynamic(Direction.kForward))
        .andThen(indexerSysIdRoutine.dynamic(Direction.kReverse))
        .andThen(indexerSysIdRoutine.quasistatic(Direction.kForward))
        .andThen(indexerSysIdRoutine.quasistatic(Direction.kReverse));
  }

  @Override
  public void simulationPeriodic() {
    indexerSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    indexerSim.setInputVoltage(indexerSimState.getMotorVoltage());
    indexerSim.update(kT);
    indexerSimState.setRawRotorPosition(indexerSim.getAngularPositionRotations());
    indexerSimState.setRotorVelocity(indexerSim.getAngularVelocityRPM() / 60.0);
  }

  @Logged
  public Voltage getIndexerVoltage() {
    return indexerMotor.getMotorVoltage().getValue();
  }

  @Logged
  public double getIndexerVelocity() {
    return indexerMotor.getVelocity().getValueAsDouble();
  }

  @Logged
  private Voltage getIndexerVoltage() {
    return indexerMotor.getMotorVoltage().getValue();
  }

  @Logged
  public double getIndexerPosition() {
    return indexerMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getSimIndexerPosition() {
    return indexerSim.getAngularPositionRotations();
  }

  @Logged
  public double getSimIndexerVelocity() {
    return indexerSim.getAngularVelocityRPM() / 60.0;
  }

  @Logged
  public Command spin() {
    return runOnce(() -> indexerMotor.setControl(indexerControl.withOutput(Volts.of(10))));
  }

  public Command stopSpin() {
    return runOnce(() -> indexerMotor.setControl(indexerControl.withOutput(Volts.of(0))));
  }
}
