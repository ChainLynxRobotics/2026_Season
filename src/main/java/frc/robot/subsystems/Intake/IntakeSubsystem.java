package frc.robot.subsystems.Intake;

import static frc.robot.subsystems.Intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeConstants.IntakeState;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {
  private TalonFX intakeSpinMotor = new TalonFX(0);
  private TalonFX intakeHeightMotor = new TalonFX(0);
  private TalonFXConfiguration intakeHeightConfiguration = new TalonFXConfiguration();
  private VelocityVoltage intakeSpinControl = new VelocityVoltage(0);

  public IntakeSubsystem() {
    this.intakeHeightConfiguration.Slot0 = intakeHeightSlot0Config;
  }

  @Logged
  public double getIntakeSpinPosition() {
    return intakeSpinMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getIntakeSpinVelocity() {
    return intakeSpinMotor.getVelocity().getValueAsDouble();
  }

  @Logged
  public double getIntakeHeightPosition() {
    return intakeHeightMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getIntakeHeightVelocity() {
    return intakeHeightMotor.getVelocity().getValueAsDouble();
  }

  public void setHeight(IntakeState state) {
    intakeHeightMotor.setPosition(state.getAngle());
  }

  public void spin() {
    intakeSpinMotor.setControl(intakeSpinControl.withVelocity(kGoalIntakeSpinVelocity));
  }

  @Override
  public void close() {
    intakeSpinMotor.close();
    intakeHeightMotor.close();
  }
}
