package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  private TalonFX spinMotor;
  private TalonFX heightMotor;

  private TalonFXConfiguration heightMotoreightConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration spinConfiguration = new TalonFXConfiguration();
  private VelocityVoltage SpinControl = new VelocityVoltage(kGoalIntakeSpinVelocity);

  private TalonFXSimState spinSimState;
  private TalonFXSimState heightSimState;

  private DCMotor x44Gearbox = DCMotor.getKrakenX44Foc(1);
  private DCMotor x60GearBox = DCMotor.getKrakenX60Foc(1);

  private DCMotorSim spinSim;
  private DCMotorSim heightSim;

  public IntakeSubsystem(TalonFX spinMotor, TalonFX heightMotor) {
    this.spinMotor = spinMotor;
    this.heightMotor = heightMotor;
    heightMotoreightConfiguration.Slot0 = kIntakeHeightSlot0Config;
    heightMotor.getConfigurator().apply(heightMotoreightConfiguration);
    spinConfiguration.Slot0 = kIntakeSpinSlot0Config;
    spinMotor.getConfigurator().apply(spinConfiguration);

    spinSimState = new TalonFXSimState(spinMotor);
    heightSimState = new TalonFXSimState(heightMotor);

    spinSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(x44Gearbox, 0.001, kSpinGearRatio), x44Gearbox);
    heightSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(x60GearBox, 0.001, kHeightGearRatio), x60GearBox);
  }

  @Override
  public void simulationPeriodic() {
    spinSimState.setSupplyVoltage(12.0);
    spinSimState.setRawRotorPosition(this.getAngularSpinPosition());
    spinSimState.setRotorVelocity(this.getAngularSpinVelocity());
    spinSim.update(kT);
    spinSim.setInputVoltage(spinSimState.getMotorVoltage());
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
  public Angle getAngularHeightPosition() {
    return heightMotor.getPosition().getValue();
  }

  @Logged
  public AngularVelocity getAngularHeightVelocity() {
    return heightMotor.getVelocity().getValue();
  }

  public void setHeight(IntakeHeightState state) {
    heightMotor.setPosition(state.getAngle());
  }

  public Command spin() {
    return runOnce(() -> spinMotor.setControl(SpinControl.withVelocity(kGoalIntakeSpinVelocity)));
  }

  @Logged
  public Angle getSpinReference() {
    return (Rotations.of(spinMotor.getClosedLoopReference().getValue()));
  }

  @Logged
  public Angle getHeightReference() {
    return (Rotations.of(heightMotor.getClosedLoopReference().getValue()));
  }

  @Override
  public void close() {
    spinMotor.close();
    heightMotor.close();
  }
}
