package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.utils.TunableNumber;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class IndexerSubsystem extends SubsystemBase {
  private TalonFX indexerMotor;
  private TalonFXConfiguration indexerConfiguration = new TalonFXConfiguration();
  private VelocityVoltage indexerControl =
      new VelocityVoltage(RotationsPerSecond.of(0)).withEnableFOC(true);

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

  private TunableNumber tunableIndexerP;
  private TunableNumber tunableIndexerI;
  private TunableNumber tunableIndexerD;
  private TunableNumber tunableIndexerS;
  private TunableNumber tunableIndexerV;
  private TunableNumber tunableIndexerVelocity;
  private TunableNumber tunableIndexerUnjamTime;
  private TunableNumber tunableIndexerRestartTime;
  @Logged public boolean isReverseIndexer;

  @Logged public boolean isSlowIndexer = false;
  private TunableNumber tunableIndexerSlowTime =
      new TunableNumber("indexerUnjamTime", kIndexerSlowTime);
  private TunableNumber tunableIndexerSpeedUpTime =
      new TunableNumber("indexerRestartTime", kIndexerSpeedUpTime);
  private TunableNumber tunableIndexerSlowSpeed =
      new TunableNumber("indexerSlowSpeed", kIndexerSlowSpeed.in(RotationsPerSecond));

  public IndexerSubsystem(TalonFX indexerMotor) {
    this.indexerMotor = indexerMotor;

    indexerConfiguration.Slot0 = kIntakeIndexerSlot0Config;
    indexerMotor.getConfigurator().apply(indexerConfiguration);

    indexerSimState = indexerMotor.getSimState();

    this.tunableIndexerP = new TunableNumber("indexerP", kIndexerP);
    this.tunableIndexerI = new TunableNumber("indexerI", kIndexerI);
    this.tunableIndexerD = new TunableNumber("indexerD", kIndexerD);
    this.tunableIndexerV = new TunableNumber("indexerV", kIndexerV);
    this.tunableIndexerS = new TunableNumber("indexerS", kIndexerS);
    this.tunableIndexerVelocity =
        new TunableNumber("indexerVelocity", kGoalIndexerVelocity.in(RotationsPerSecond));
    this.isReverseIndexer = false;
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

  private TalonFXConfiguration generateTunableIndexerConfig() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(tunableIndexerP.get())
                    .withKI(tunableIndexerI.get())
                    .withKD(tunableIndexerD.get())
                    .withKV(tunableIndexerV.get())
                    .withKS(tunableIndexerS.get()));
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = kIndexerGearRatio;
    return config;
  }

  public void detectTunableIndexerChanges() {
    if (tunableIndexerP.hasChanged()
        || tunableIndexerI.hasChanged()
        || tunableIndexerD.hasChanged()) {
      this.indexerMotor.getConfigurator().apply(generateTunableIndexerConfig());
    }
  }

  @Override
  public void periodic() {
    detectTunableIndexerChanges();
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

  public double getIndexerUnjamTime() {
    return tunableIndexerUnjamTime.get();
  }

  public double getIndexerRestartTime() {
    return tunableIndexerRestartTime.get();
  }

  public double getIndexerSlowTime() {
    return tunableIndexerSlowTime.get();
  }

  public double getIndexerSpeedUpTime() {
    return tunableIndexerSpeedUpTime.get();
  }

  public void swapIndexerDir() {
    if (isReverseIndexer) {
      isReverseIndexer = false;
    } else {
      isReverseIndexer = true;
    }
  }

  public Pose3d getIndexerPose() {
    return new Pose3d(
        0,
        0,
        0.02,
        new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(getIndexerPosition() * 360)));
  }

  public Command spin() {
    return run(() -> spinInternal());
  }

  public void spinInternal() {
    double indexerVelocity =
        isSlowIndexer ? tunableIndexerSlowSpeed.get() : tunableIndexerVelocity.get();
    indexerMotor.setControl(
        indexerControl.withVelocity(
            isReverseIndexer
                ? RotationsPerSecond.of(indexerVelocity * -1)
                : RotationsPerSecond.of(indexerVelocity)));
  }

  public Command spin10V() {
    return runOnce(() -> indexerMotor.setControl(new VoltageOut(Volts.of(10))));
  }

  public Command stopSpin() {
    return runOnce(() -> indexerMotor.setControl(new VoltageOut(Volts.of(0))));
  }

  public void stopSpinInternal() {
    indexerMotor.setControl(new VoltageOut(Volts.of(0)));
  }
}
