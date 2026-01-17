package frc.robot.subsystems.Serializer;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SerializerSubsystem extends SubsystemBase {
  private TalonFX serializerMotor = new TalonFX(0);

  public SerializerSubsystem() {}

  @Logged
  public double getSerializerVelocity() {
    return serializerMotor.getVelocity().getValueAsDouble();
  }

  @Logged
  public double getSerializerPosition() {
    return serializerMotor.getPosition().getValueAsDouble();
  }

  public void spin() {
    serializerMotor.setVoltage(0);
  }
}
