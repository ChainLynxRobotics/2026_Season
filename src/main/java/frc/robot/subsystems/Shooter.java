package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.ShooterConstants.*;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Shooter extends SubsystemBase implements AutoCloseable {
  private final TalonFX flywheelMotor;
  private final MotionMagicVelocityVoltage flywheelMotionMagic;
  private FlywheelSim flywheelSim = null;

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

    this.flywheelSim = new FlywheelSim(null, DCMotor.getKrakenX60Foc(1), null);
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

  public AngularVelocity getFlywheelVelocity() {
    return flywheelMotor.getVelocity().getValue();
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

  final MotionMagicVoltage request = new MotionMagicVoltage(0);

  public Command func1(Angle position) {
    return runOnce(() -> hoodMotor.setControl(request.withPosition(position)));
  }

  @Override
  public void periodic() {
    if (hoodLimitSwitch.get()) {
      hoodMotor.setPosition(Degrees.of(90));
    }
  }

  @Override
  public void simulationPeriodic() {}
}
