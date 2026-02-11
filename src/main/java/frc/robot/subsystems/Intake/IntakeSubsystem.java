package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeConstants.IntakeHeightState;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  private double setpoint = 0;

  private TalonFX spinMotor;
  private TalonFX heightMotor;

  private TalonFXConfiguration heightMotorConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration spinConfiguration = new TalonFXConfiguration();

  private VelocityVoltage spinControl =
      new VelocityVoltage(
          (kGoalIntakeSpinVelocity.in(RotationsPerSecond) * kInputToOutputSpinGearRatio));
  private MotionMagicVoltage heightControl = new MotionMagicVoltage(0.0);

  private TalonFXSimState spinSimState;
  private TalonFXSimState heightSimState;

  private DCMotor x44Gearbox = DCMotor.getKrakenX44Foc(1);
  private DCMotor x60GearBox = DCMotor.getKrakenX60Foc(1);

  private DCMotorSim spinSim;
  private SingleJointedArmSim heightSim;

  public IntakeSubsystem(TalonFX spinMotor, TalonFX heightMotor) {
    this.spinMotor = spinMotor;
    this.heightMotor = heightMotor;
    // heightMotorConfiguration.Slot0 = kIntakeHeightSlot0Config;
    // heightMotorConfiguration.MotionMagic = kIntakeHeightMotionMagic;
    //heightMotorConfiguration.Feedback.SensorToMechanismRatio = kInputToOutputHeightGearRatio;
    heightMotor.getConfigurator().apply(heightMotorConfiguration);
    spinConfiguration.Slot0 = kIntakeSpinSlot0Config;
    spinConfiguration.Feedback.SensorToMechanismRatio = kInputToOutputSpinGearRatio;
    spinMotor.getConfigurator().apply(spinConfiguration);

    spinSimState = new TalonFXSimState(spinMotor);
    heightSimState = new TalonFXSimState(heightMotor);

    spinSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(x44Gearbox, 0.001, 1 / kInputToOutputSpinGearRatio),
            x44Gearbox);
    heightSim =
        new SingleJointedArmSim(
            x60GearBox,
            kInputToOutputHeightGearRatio,
            0.16564482161292,
            kIntakeLengthFromPivot,
            IntakeHeightState.LOW.getAngle().in(Radians),
            IntakeHeightState.HIGH.getAngle().in(Radians),
            false,
            IntakeHeightState.LOW.getAngle().in(Radians));
  }

  @Override
  public void simulationPeriodic() {
    spinSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    spinSim.setInputVoltage(spinSimState.getMotorVoltage());
    spinSim.update(kT);
    spinSimState.setRawRotorPosition(
        spinSim.getAngularPositionRotations() / kInputToOutputSpinGearRatio);
    spinSimState.setRotorVelocity(
        spinSim.getAngularVelocityRPM() / 60.0 / kInputToOutputSpinGearRatio);

    heightSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    heightSim.setInputVoltage(heightSimState.getMotorVoltage());
    heightSim.update(kT);
    heightSimState.setRawRotorPosition(
        heightSim.getAngleRads() / 2 / Math.PI / kInputToOutputHeightGearRatio);
    heightSimState.setRotorVelocity(
        heightSim.getVelocityRadPerSec() / 2 / Math.PI / kInputToOutputHeightGearRatio);
  }

  @Logged
  public double getSimulatedBatteryVoltage() {
    return SimulatedBattery.getBatteryVoltage().in(Volts);
  }

  @Logged
  public double getSpinPosition() {
    return spinMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getSpinVelocity() {
    return spinMotor.getVelocity().getValueAsDouble();
  }

  @Logged
  public double getSimSpinPosition() {
    return spinSim.getAngularPositionRotations();
  }

  @Logged
  public double getSimSpinVelocity() {
    return spinSim.getAngularVelocityRPM() / 60.0;
  }

  @Logged
  public Angle getAngularSpinPosition() {
    return spinMotor.getPosition().getValue();
  }

  @Logged
  public AngularVelocity getAngularSpinVelocity() {
    return spinMotor.getVelocity().getValue();
  }

  @Logged
  public double getHeightPosition() {
    return heightMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getHeightVelocity() {
    return heightMotor.getVelocity().getValueAsDouble();
  }

  @Logged
  public double getSimHeightPosition() {
    return heightSim.getAngleRads() / 2 / Math.PI;
  }

  @Logged
  public double getSimHeightVelocity() {
    return heightSim.getVelocityRadPerSec() / 2 / Math.PI;
  }

  @Logged
  public Angle getAngularHeightPosition() {
    return heightMotor.getPosition().getValue();
  }

  @Logged
  public AngularVelocity getAngularHeightVelocity() {
    return heightMotor.getVelocity().getValue();
  }

  @Logged
  public double getSetpoint() {
    return setpoint;
  }

  @Logged
  public double getSimVoltage() {
    return heightSimState.getMotorVoltage();
  }

  public Command spin() {
    return runOnce(() -> spinMotor.setControl(spinControl));
  }

  public Command setHeight(IntakeHeightState state) {
    return runOnce(
        () -> {
          setpoint = state.getAngle().in(Rotations);
          heightMotor.set(1);
          // setControl(heightControl.withPosition(state.getAngle().in(Rotations)));
        });
  }

  @Logged
  public double getHeightReference() {
    return heightMotor.getClosedLoopReference().getValue();
  }

  @Override
  public void close() {
    spinMotor.close();
    heightMotor.close();
  }
}
