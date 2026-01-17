package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;
import frc.robot.utils.RobotMath;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class Climber extends SubsystemBase implements AutoCloseable {
  // setpoint for checking against encoder
  private Angle setpoint;
  private TalonFX motor;
  private DCMotorSim motorSim;
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private TalonFXSimState simMotor;
  private DigitalInput limitSwitch;
  private boolean doMotionProfiling;

  public Climber(TalonFX motor) {
    this.motor = motor;
    TalonFXSimState simMotor = new TalonFXSimState(motor); // create our sim Talon
    DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, kGearRatio), gearbox);

    limitSwitch = new DigitalInput(1);

    TalonFXConfiguration talonFXConfigs =
        new TalonFXConfiguration()
            .withSlot0(slot0Configs)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    motor.getConfigurator().apply(talonFXConfigs);
    doMotionProfiling = true;
  }

  @Override
  public void periodic() {
    if (doMotionProfiling) {
      motor.setControl(request.withPosition(setpoint));
    }
  }

  // takes in state and compares with map to get angle value, and sets motionmagic to angle setpoint
  public void setStateSetpoint(ClimberState state) {
    setpoint = climberMap.get(state);
    doMotionProfiling = true;
  }

  public void setFullPower() {
    doMotionProfiling = false;
    motor.set(1);
  }

  public Angle getSetpoint() {
    return setpoint;
  }

  // stop motor from moving and enter break mode by default
  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void close() {
    motor.close();
    limitSwitch.close();
  }

  // raw rotations before gear ratio
  public Angle getPosition() {
    return (motor.getPosition().getValue());
  }
  // checks internal encoder to setpoint + and - a certain tolerance
  public boolean atSetpoint() {
    return RobotMath.measureWithinBounds(
        getPosition(), setpoint.minus(kSetpointTolerance), setpoint.plus(kSetpointTolerance));
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  @Override
  public void simulationPeriodic() {
    simMotor.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    motorSim.setInputVoltage(simMotor.getMotorVoltage());

    motorSim.update(kDT.in(Seconds));

    simMotor.setRawRotorPosition(motorSim.getAngularPosition());
    simMotor.setRotorVelocity(motorSim.getAngularVelocity());
    simMotor.setRotorAcceleration(motorSim.getAngularAcceleration());
  }
}
