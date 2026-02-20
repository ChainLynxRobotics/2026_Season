package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Intake.IntakeConstants.IntakeHeightState;
import frc.robot.utils.TunableNumber;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

@Logged
public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  private double setpoint = 0;

  private TalonFX spinMotor;
  private TalonFX heightMotor;

  private TalonFXConfiguration spinConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration heightMotorConfiguration = new TalonFXConfiguration();

  private VelocityVoltage spinControl = new VelocityVoltage(kGoalIntakeSpinVelocity);
  private MotionMagicVoltage heightControl = new MotionMagicVoltage(0.0);

  private TalonFXSimState spinSimState;
  private TalonFXSimState heightSimState;

  private DCMotor x44Gearbox = DCMotor.getKrakenX44Foc(1);
  private DCMotor x60GearBox = DCMotor.getKrakenX60Foc(1);

  private TunableNumber tunableIntakeSpinP;
  private TunableNumber tunableIntakeSpinI;
  private TunableNumber tunableIntakeSpinD;
  private TunableNumber tunableIntakeSpinVelocity;

  private DCMotorSim spinSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(x44Gearbox, 0.001, 1), x44Gearbox);

  private SingleJointedArmSim heightSim =
      new SingleJointedArmSim(
          x60GearBox,
          // kInputToOutputHeightGearRatio,
          1,
          0.16564482161292,
          kIntakeLengthFromPivot,
          IntakeHeightState.HIGH.getAngle().in(Radians),
          10000,
          false,
          IntakeHeightState.HIGH.getAngle().in(Radians));

  private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private SysIdRoutine spinSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(0.5).per(Seconds), Volts.of(2), Seconds.of(6)),
          new SysIdRoutine.Mechanism(
              volts -> spinMotor.setControl(voltageOut.withOutput(volts.in(Volts))),
              log ->
                  log.motor("Leader")
                      .angularPosition(Rotations.of(this.getSpinPosition()))
                      .angularVelocity(RotationsPerSecond.of(this.getSpinVelocity()))
                      .voltage(this.getSpinVoltage()),
              this));

  private SysIdRoutine heightSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.of(0.5).per(Seconds), Volts.of(2), Seconds.of(6)),
          new SysIdRoutine.Mechanism(
              volts -> heightMotor.setControl(voltageOut.withOutput(volts.in(Volts))),
              log ->
                  log.motor("Leader")
                      .angularPosition(Rotations.of(this.getHeightPosition()))
                      .angularVelocity(RotationsPerSecond.of(this.getHeightVelocity()))
                      .voltage(this.getHeightVoltage()),
              this));

  public IntakeSubsystem(TalonFX spinMotor, TalonFX heightMotor) {
    this.spinMotor = spinMotor;
    spinConfiguration.Slot0 = kIntakeSpinSlot0Config;
    spinConfiguration.Feedback.SensorToMechanismRatio = kInputToOutputSpinGearRatio;
    spinConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    spinMotor.getConfigurator().apply(spinConfiguration);

    this.heightMotor = heightMotor;
    heightMotorConfiguration.Slot0 = kIntakeHeightSlot0Config;
    heightMotorConfiguration.MotionMagic = kIntakeHeightMotionMagic;
    heightMotorConfiguration.Feedback.SensorToMechanismRatio = kInputToOutputHeightGearRatio;
    heightMotor.getConfigurator().apply(heightMotorConfiguration);

    spinSimState = spinMotor.getSimState();
    heightSimState = heightMotor.getSimState();

    this.tunableIntakeSpinP = new TunableNumber("IntakeSpinP", kSpinP);
    this.tunableIntakeSpinI = new TunableNumber("IntakeSpinI", kSpinI);
    this.tunableIntakeSpinD = new TunableNumber("IntakeSpinD", kSpinD);
    this.tunableIntakeSpinVelocity =
        new TunableNumber("IntakeSpinVelocity", kGoalIntakeSpinVelocity.in(RotationsPerSecond));
  }

  public void runHeightSysId() {
    run(() -> heightSysIdRoutine.dynamic(Direction.kForward))
        .andThen(heightSysIdRoutine.dynamic(Direction.kReverse))
        .andThen(heightSysIdRoutine.quasistatic(Direction.kForward))
        .andThen(heightSysIdRoutine.quasistatic(Direction.kReverse));
  }

  public void runSpinSysId() {
    run(() -> spinSysIdRoutine.dynamic(Direction.kForward))
        .andThen(spinSysIdRoutine.dynamic(Direction.kReverse))
        .andThen(spinSysIdRoutine.quasistatic(Direction.kForward))
        .andThen(spinSysIdRoutine.quasistatic(Direction.kReverse));
  }

  private Voltage getSpinVoltage() {
    return spinMotor.getMotorVoltage().getValue();
  }

  private Voltage getHeightVoltage() {
    return heightMotor.getMotorVoltage().getValue();
  }

  @Override
  public void simulationPeriodic() {
    spinSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    spinSim.setInputVoltage(spinSimState.getMotorVoltage());
    spinSim.update(kT);
    spinSimState.setRawRotorPosition(spinSim.getAngularPositionRotations());
    spinSimState.setRotorVelocity(spinSim.getAngularVelocityRPM() / 60.0);

    heightSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    heightSim.setInputVoltage(heightSimState.getMotorVoltage());
    heightSim.update(kT);
    heightSimState.setRawRotorPosition(heightSim.getAngleRads() / 2 / Math.PI);
    heightSimState.setRotorVelocity(heightSim.getVelocityRadPerSec() / 2 / Math.PI);
  }

  public double getSpinPosition() {
    return spinMotor.getPosition().getValueAsDouble();
  }

  public double getSpinVelocity() {
    return spinMotor.getVelocity().getValueAsDouble();
  }

  public double getSimSpinPosition() {
    return spinSim.getAngularPositionRotations();
  }

  public double getSimSpinVelocity() {
    return spinSim.getAngularVelocityRPM() / 60.0;
  }

  public double getHeightPosition() {
    return heightMotor.getPosition().getValueAsDouble();
  }

  public double getHeightVelocity() {
    return heightMotor.getVelocity().getValueAsDouble();
  }

  public double getSimHeightPosition() {
    return heightSim.getAngleRads() / 2 / Math.PI;
  }

  public double getSimHeightVelocity() {
    return heightSim.getVelocityRadPerSec() / 2 / Math.PI;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public Command spin() {
    return runOnce(
        () ->
            spinMotor.setControl(
                spinControl.withVelocity(RotationsPerSecond.of(tunableIntakeSpinVelocity.get()))));
  }

  public Command spin5V() {
    return runOnce(() -> spinMotor.setControl(new VoltageOut(Volts.of(5))));
  }

  public Command stopSpin() {
    return runOnce(() -> spinMotor.setControl(voltageOut.withOutput(0)));
  }

  public Command setHeight(IntakeHeightState state) {
    return runOnce(
        () -> {
          setpoint = state.getAngle().in(Rotations);
          heightMotor.setControl(heightControl.withPosition(state.getAngle().in(Rotations)));
        });
  }

  public double getHeightReference() {
    return heightMotor.getClosedLoopReference().getValue();
  }

  @Override
  public void close() {
    spinMotor.close();
    heightMotor.close();
  }

  private TalonFXConfiguration generateTunableIntakeSpinConfig() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(tunableIntakeSpinP.get())
                    .withKI(tunableIntakeSpinI.get())
                    .withKD(tunableIntakeSpinD.get()));
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = kInputToOutputSpinGearRatio;
    return config;
  }

  public void detectTunableIntakeSpinChanges() {
    if (tunableIntakeSpinP.hasChanged()
        || tunableIntakeSpinI.hasChanged()
        || tunableIntakeSpinD.hasChanged()) {
      this.spinMotor.getConfigurator().apply(generateTunableIntakeSpinConfig());
    }
  }

  @Override
  public void periodic() {
    detectTunableIntakeSpinChanges();
  }
}
