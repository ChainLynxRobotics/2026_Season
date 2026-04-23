package frc.robot.subsystems.Serializer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Serializer.SerializerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class SerializerSubsystem extends SubsystemBase {
  private TalonFX serializerMotor;
  private TalonFXConfiguration serializerConfiguration = new TalonFXConfiguration();
  private VelocityVoltage serializerControl = new VelocityVoltage(RotationsPerSecond.of(0));

  private DCMotor x44Gearbox = DCMotor.getKrakenX44Foc(1);
  private TalonFXSimState serializerSimState;
  private DCMotorSim serializerSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(x44Gearbox, 0.001, 1), x44Gearbox);

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private SysIdRoutine serializerSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(0.5).per(Seconds), Volts.of(2), Seconds.of(6)),
          new SysIdRoutine.Mechanism(
              volts -> serializerMotor.setControl(voltageOut.withOutput(volts.in(Volts))),
              log ->
                  log.motor("Leader")
                      .angularPosition(Rotations.of(this.getSerializerPosition()))
                      .angularVelocity(RotationsPerSecond.of(this.getSerializerVelocity()))
                      .voltage(this.getSerializerVoltage()),
              this));

  public SerializerSubsystem(TalonFX serializerMotor) {
    this.serializerMotor = serializerMotor;

    serializerConfiguration.Slot0 = kIntakeSerializerSlot0Config;
    serializerConfiguration.Audio.AllowMusicDurDisable = true;
    serializerMotor.getConfigurator().apply(serializerConfiguration);

    serializerSimState = serializerMotor.getSimState();
  }

  public void runSysId() {
    run(() -> serializerSysIdRoutine.dynamic(Direction.kForward))
        .andThen(serializerSysIdRoutine.dynamic(Direction.kReverse))
        .andThen(serializerSysIdRoutine.quasistatic(Direction.kForward))
        .andThen(serializerSysIdRoutine.quasistatic(Direction.kReverse));
  }

  @Override
  public void simulationPeriodic() {
    serializerSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    serializerSim.setInputVoltage(serializerSimState.getMotorVoltage());
    serializerSim.update(kT);
    serializerSimState.setRawRotorPosition(serializerSim.getAngularPositionRotations());
    serializerSimState.setRotorVelocity(serializerSim.getAngularVelocityRPM() / 60.0);
  }

  @Logged
  public Voltage getSerializerVoltage() {
    return serializerMotor.getMotorVoltage().getValue();
  }

  @Logged
  public double getSerializerPosition() {
    return serializerMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getSerializerVelocity() {
    return serializerMotor.getVelocity().getValueAsDouble();
  }

  @Logged
  public double getSimSerializerPosition() {
    return serializerSim.getAngularPositionRotations();
  }

  @Logged
  public double getSimSerializerVelocity() {
    return serializerSim.getAngularVelocityRPM() / 60.0;
  }

  @Logged
  public Command spin() {
    return run(() -> spinInternal()).withName("Spin");
  }

  public void spinInternal() {
    serializerMotor.setControl(voltageOut.withOutput(Volts.of(10)));
  }

  @Logged
  public Command stopSpin() {
    return runOnce(() -> stopSpinInternal()).withName("Stop spin");
  }

  public void stopSpinInternal() {
    serializerMotor.setControl(voltageOut.withOutput(Volts.of(0)));
  }
}
