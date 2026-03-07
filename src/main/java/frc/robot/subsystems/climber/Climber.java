package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;
import frc.robot.utils.RobotMath;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

@Logged
public class Climber extends SubsystemBase implements AutoCloseable {
  // setpoint for checking against encoder
  private Distance setpoint;
  private TalonFX motor;
  private ElevatorSim motorSim;
  private final PositionVoltage request = new PositionVoltage(0);
  private TalonFXSimState simMotor;
  private DigitalInput limitSwitch;

  public Climber(TalonFX motor) {
    this.motor = motor;
    this.simMotor = new TalonFXSimState(motor); // create our sim Talon
    DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    motorSim =
        new ElevatorSim(
            gearbox, kGearRatio, (70), kSpoolRadius.in(Meters), 0, kMaxHeight.in(Meters), true, 0);

    limitSwitch = new DigitalInput(kLimitSwitchId);

    TalonFXConfiguration talonFXConfigs =
        new TalonFXConfiguration()
            .withSlot0(slot0Configs)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 100;
    motor.getConfigurator().apply(talonFXConfigs);
    motor.setPosition(Degree.of(0));
    setpoint = climberMap.get(ClimberState.BOTTOM);
  }

  @Override
  public void periodic() {
    // Limit switch not wired properly
    // if (Inches.of(getHeightInches()).gt(kMaxHeight.minus(Inches.of(-2))) && limitSwitch.get()) {
    //   motor.setPosition(ClimberConstants.convertToAngle(kMaxHeight));
    // }
  }

  // takes in state and compares with map to get angle value, and sets motionmagic to angle setpoint
  public void setStateSetpoint(ClimberState state) {
    setpoint = climberMap.get(state);
    motor.setControl(request.withPosition(ClimberConstants.convertToAngle(setpoint)));
  }

  public Distance getSetpoint() {
    return setpoint;
  }

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
    return motor.getPosition().getValue();
  }

  public double getHeightInches() {
    return (ClimberConstants.convertToHeight(getPosition()).in(Inches));
  }

  public Voltage getVoltage() {
    return motor.getMotorVoltage().getValue();
  }

  public Current getStatorCurrent() {
    return motor.getStatorCurrent().getValue();
  }

  public Current getSupplyCurrent() {
    return motor.getSupplyCurrent().getValue();
  }

  // checks internal encoder to setpoint + and - a certain tolerance
  public boolean atSetpoint() {
    return RobotMath.measureWithinBounds(
        Inches.of(getHeightInches()),
        setpoint.minus(kSetpointTolerance),
        setpoint.plus(kSetpointTolerance));
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public void zeroClimber() {
    motor.setPosition(Rotations.zero());
  }

  public Command goToStateCommand(ClimberState state) {
    return run(() -> setStateSetpoint(state));
    // .until(() -> limitSwitch.get())
    // .andThen(() -> stopMotor());
  }

  @Override
  public void simulationPeriodic() {
    simMotor.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    motorSim.setInputVoltage(simMotor.getMotorVoltage());

    motorSim.update(kDT.in(Seconds));

    simMotor.setRawRotorPosition(
        ClimberConstants.convertToAngle(Meters.of(motorSim.getPositionMeters())));
    simMotor.setRotorVelocity(
        RotationsPerSecond.of(
            (ClimberConstants.convertToAngle(Meters.of(motorSim.getVelocityMetersPerSecond()))
                .in(Rotations))));
  }
}
