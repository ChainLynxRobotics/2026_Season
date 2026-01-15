package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.ShooterConstants.*;
import static frc.robot.utils.RobotMath.*;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

@Logged
public class Shooter extends SubsystemBase implements AutoCloseable {
  private final TalonFX flywheelMotor;
  private final MotionMagicVelocityVoltage flywheelMotionMagic;
  private DCMotorSim flywheelSim = null;
  private TalonFXSimState flywheelMotorSim;

  private final TalonFX hoodMotor;
  private final DigitalInput hoodLimitSwitch;

  public Shooter() {
    this.hoodLimitSwitch = new DigitalInput(kHoodLimitSwitchId);

    this.flywheelMotor = new TalonFX(kFlywheelCANId);
    flywheelMotor.getConfigurator().apply(kFlyWheelConfig);
    flywheelMotionMagic =
        new MotionMagicVelocityVoltage(RotationsPerSecond.zero()).withEnableFOC(true);

    this.hoodMotor = new TalonFX(kHoodCANId);
    hoodMotor.getConfigurator().apply(kHoodConfig);

    if (RobotBase.isReal()) return;

    this.flywheelMotorSim = flywheelMotor.getSimState();
    this.flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                kFlywheelMOI.in(KilogramSquareMeters),
                kFlywheelGearRatio),
            DCMotor.getKrakenX60Foc(1));
  }

  public void stop() {
    flywheelMotor.stopMotor();
    hoodMotor.stopMotor();
  }

  @Override
  public void close() {
    flywheelMotor.close();
    hoodMotor.close();
  }

  public Angle getFlywheelPosition() {
    return flywheelMotor.getPosition().getValue();
  }

  public double getFlywheelPositionRotations() {
    return getFlywheelPosition().in(Rotations);
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheelMotor.getVelocity().getValue();
  }

  public double getFlywheelVelocityRpS() {
    return getFlywheelVelocity().in(RotationsPerSecond);
  }

  public Voltage getFlywheelVoltage() {
    return flywheelMotor.getMotorVoltage().getValue();
  }

  public ControlModeValue getControlMode() {
    return flywheelMotor.getControlMode().getValue();
  }

  public Command setFlywheelVelocity(AngularVelocity velocity) {
    return runOnce(() -> flywheelMotor.setControl(flywheelMotionMagic.withVelocity(velocity)));
  }

  public Angle getHoodPosition() {
    return hoodMotor.getPosition().getValue();
  }

  public Command homeHood() {
    Command command = run(() -> hoodMotor.set(0.01)).until(hoodLimitSwitch::get);
    command.addRequirements(this);
    return command;
  }

  public Command flywheelSysid() {
    var routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(6.5), null, null),
            new SysIdRoutine.Mechanism(this::flywheelVoltageDrive, null, this));
    return sequence(
        routine
            .quasistatic(Direction.kForward)
            .until(() -> getFlywheelVelocity().gt(RotationsPerSecond.of(50))),
        routine
            .quasistatic(Direction.kReverse)
            .until(() -> getFlywheelVelocity().lt(RotationsPerSecond.zero())),
        routine
            .dynamic(Direction.kForward)
            .until(() -> getFlywheelVelocity().gt(RotationsPerSecond.of(50))),
        routine
            .dynamic(Direction.kReverse)
            .until(() -> getFlywheelVelocity().lt(RotationsPerSecond.zero())));
  }

  private void flywheelVoltageDrive(Voltage voltage) {
    flywheelMotor.setControl(new VoltageOut(voltage));
  }

  final MotionMagicVoltage request = new MotionMagicVoltage(0);

  public Command func1(Angle position) {
    return runOnce(() -> hoodMotor.setControl(request.withPosition(position)));
  }

  @Override
  public void periodic() {
    if (hoodLimitSwitch.get()
        && RobotBase.isReal()
        && isWithinTolerance(getHoodPosition(), Degrees.of(90), Degrees.of(0.01))) {
      hoodMotor.setPosition(Degrees.of(90));
    }
  }

  @Override
  public void simulationPeriodic() {
    flywheelMotorSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    flywheelSim.setInputVoltage(flywheelMotorSim.getMotorVoltage());

    flywheelSim.update(kDT.in(Seconds));

    flywheelMotorSim.setRawRotorPosition(flywheelSim.getAngularPosition());
    flywheelMotorSim.setRotorVelocity(flywheelSim.getAngularVelocity());
    flywheelMotorSim.setRotorAcceleration(flywheelSim.getAngularAcceleration());
  }
}
