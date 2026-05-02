package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class TuneableTalonFX {
  private final TalonFX motor;
  private TalonFXSimState simState = null;
  private LinearSystemSim<N2, N1, N2> physicsSim = null;

  private final TunableNumber kG;
  private final TunableNumber kS;
  private final TunableNumber kA;
  private final TunableNumber kV;
  private final TunableNumber kP;
  private final TunableNumber kI;
  private final TunableNumber kD;

  private final TunableNumber supplyCurrentLimit;
  private final TunableNumber statorCurrentLimit;

  private final TunableNumber motionMagicCruiseVelocity;
  private final TunableNumber motionMagicAcceleration;
  private final TunableNumber motionMagicJerk;
  private final TunableNumber motionMagicExpoKA;
  private final TunableNumber motionMagicExpoKV;

  private TalonFXConfiguration config;

  private StatusCode status;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Double> setpoint;
  private final StatusSignal<ControlModeValue> controlMode;

  /**
   * @param CANId the CAN id of the talon fx
   * @param name the name of the motor to be shown on network tables / elastic
   * @param canBus the CAN bus the talon fx is on
   * @param config the config you want to
   * @param physicsSim
   */
  public TuneableTalonFX(
      int CANId,
      String name,
      CANBus canBus,
      TalonFXConfiguration config,
      LinearSystemSim<N2, N1, N2> physicsSim) {
    motor = new TalonFX(CANId, canBus);

    kG = new TunableNumber(name + "/kG", config.Slot0.kG);
    kS = new TunableNumber(name + "/kS", config.Slot0.kS);
    kA = new TunableNumber(name + "/kA", config.Slot0.kA);
    kV = new TunableNumber(name + "/kV", config.Slot0.kV);
    kP = new TunableNumber(name + "/kP", config.Slot0.kP);
    kI = new TunableNumber(name + "/kI", config.Slot0.kI);
    kD = new TunableNumber(name + "/kD", config.Slot0.kD);

    supplyCurrentLimit =
        new TunableNumber(name + "/supply current limit", config.CurrentLimits.SupplyCurrentLimit);
    statorCurrentLimit =
        new TunableNumber(name + "/stator current limit", config.CurrentLimits.StatorCurrentLimit);

    motionMagicCruiseVelocity =
        new TunableNumber(
            name + "/motion magic cruse velocity", config.MotionMagic.MotionMagicCruiseVelocity);
    motionMagicAcceleration =
        new TunableNumber(
            name + "/motion magic acceleration", config.MotionMagic.MotionMagicAcceleration);
    motionMagicJerk =
        new TunableNumber(name + "/motion magic jerk", config.MotionMagic.MotionMagicJerk);
    motionMagicExpoKA =
        new TunableNumber(name + "/motion magic expo kA", config.MotionMagic.MotionMagicExpo_kA);
    motionMagicExpoKV =
        new TunableNumber(name + "/motion magic expo kV", config.MotionMagic.MotionMagicExpo_kV);

    this.config = config.clone();
    // always enable current limits so they can be tuned
    this.config.CurrentLimits.StatorCurrentLimitEnable = true;
    this.config.CurrentLimits.SupplyCurrentLimitEnable = true;

    motor.getConfigurator().apply(this.config);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    voltage = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    statorCurrent = motor.getStatorCurrent();
    setpoint = motor.getClosedLoopReference();
    controlMode = motor.getControlMode();

    if (RobotBase.isReal()) return;
    simState = motor.getSimState();
    this.physicsSim = physicsSim;
  }

  public void update() {
    status =
        BaseStatusSignal.refreshAll(
            position,
            velocity,
            acceleration,
            setpoint,
            controlMode,
            voltage,
            supplyCurrent,
            statorCurrent);

    if (checkForConfigChanges()) {
      updateConfig();
    }
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }
  }

  private void simulationPeriodic() {}

  private boolean checkForConfigChanges() {
    return kG.hasChanged()
        || kS.hasChanged()
        || kA.hasChanged()
        || kV.hasChanged()
        || kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()
        || supplyCurrentLimit.hasChanged()
        || statorCurrentLimit.hasChanged()
        || motionMagicCruiseVelocity.hasChanged()
        || motionMagicAcceleration.hasChanged()
        || motionMagicJerk.hasChanged()
        || motionMagicExpoKA.hasChanged()
        || motionMagicExpoKV.hasChanged();
  }

  private void updateConfig() {
    var slot0 = config.Slot0;
    slot0.kG = kG.get();
    slot0.kS = kS.get();
    slot0.kA = kA.get();
    slot0.kV = kV.get();
    slot0.kP = kP.get();
    slot0.kI = kI.get();
    slot0.kD = kD.get();

    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.get();
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.get();

    var motionMagic = config.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = motionMagicCruiseVelocity.get();
    motionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
    motionMagic.MotionMagicJerk = motionMagicJerk.get();
    motionMagic.MotionMagicExpo_kA = motionMagicExpoKA.get();
    motionMagic.MotionMagicExpo_kV = motionMagicExpoKV.get();

    motor.getConfigurator().apply(config);
  }

  public TalonFX getTalonFX() {
    return motor;
  }

  public void applyRequest(ControlRequest request) {
    motor.setControl(request);
  }

  public StatusCode getStatus() {
    return status;
  }

  public String getStatusString() {
    return status.getName();
  }

  public Angle getPosition() {
    return position.getValue();
  }

  public AngularVelocity getVelocity() {
    return velocity.getValue();
  }

  public AngularAcceleration getAcceleration() {
    return acceleration.getValue();
  }

  public Voltage getVoltage() {
    return voltage.getValue();
  }

  public Current getSupplyCurrent() {
    return supplyCurrent.getValue();
  }

  public Current getStatorCurrent() {
    return statorCurrent.getValue();
  }

  public double getSetpoint() {
    return setpoint.getValueAsDouble();
  }

  public ControlModeValue getControlMode() {
    return controlMode.getValue();
  }

  public String getControlModeString() {
    return getControlMode().toString();
  }
}
