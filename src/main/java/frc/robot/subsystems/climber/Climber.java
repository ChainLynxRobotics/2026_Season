package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;

public class Climber extends SubsystemBase {
  private Angle setpoint;
  private TalonFX motor = new TalonFX(kClimberId, "blinky");
  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  public Climber() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotionMagicConfigs motionMagicConfigs =
        talonFXConfigs
            .MotionMagic
            .withMotionMagicAcceleration(kCruiseAcceleration)
            .withMotionMagicCruiseVelocity(kCruiseVelocity);
    motor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {}

  public void goToState(ClimberState state) {
    motor.setControl(request.withPosition(climberMap.get(state)));
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  public Angle getPosition() {
    return (motor.getPosition().getValue());
  }

  public boolean atSetpoint() {
    return getPosition().gte(setpoint);
  }
}
