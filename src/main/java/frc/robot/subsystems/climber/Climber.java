package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;
import frc.robot.utils.RobotMath;

public class Climber extends SubsystemBase {
  // setpoint for checking against encoder
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
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {}

  // takes in state and compares with map to get angle value, and sets motionmagic to angle setpoint
  public void goToState(ClimberState state) {
    setpoint = climberMap.get(state);
    motor.setControl(request.withPosition(setpoint));
  }
  // stop motor from moving and enter break mode by default
  public void stopMotor() {
    motor.stopMotor();
  }

  public Angle getPosition() {
    return (motor.getPosition().getValue());
  }
  // checks internal encoder to setpoint + and - a certain tolerance
  public boolean atSetpoint() {
    return RobotMath.measureWithinBounds(getPosition(), setpoint.minus(kSetpointTolerance), setpoint.plus(kSetpointTolerance));
  }
}
