package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  private TalonFX indexerChuteMotor = new TalonFX(0);
  private TalonFX indexerFeedMotor = new TalonFX(0);

  public IndexerSubsystem() {}

  @Logged
  public double getIndexerChutePosition() {
    return indexerChuteMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getIndexerChuteVelocity() {
    return indexerChuteMotor.getVelocity().getValueAsDouble();
  }

  @Logged
  public double getIndexerFeedPosition() {
    return indexerFeedMotor.getPosition().getValueAsDouble();
  }

  @Logged
  public double getIndexerFeedVelocity() {
    return indexerFeedMotor.getVelocity().getValueAsDouble();
  }

  public Command spin() {
    return Commands.parallel(
        run(() -> indexerChuteMotor.setVoltage(0)), run(() -> indexerFeedMotor.setVoltage(0)));
  }
}
