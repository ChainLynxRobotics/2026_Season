package frc.robot.utils;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;

public class CurrentSpikeDetector {
  private double totalCurrent;
  private double lastCurrent;
  private int iterations = 0;

  TalonFX talonFXMotor;

  //   public CurrentSpikeDetector(TalonFX motor) {
  //     this.talonFXMotor = motor;
  //     }

  public void recordCurrent(TalonFX motor) {
    iterations++;
    lastCurrent = motor.getStatorCurrent().getValue().in(Amps);
    totalCurrent += lastCurrent;
  }

  public double getAverageCurrent() {
    return totalCurrent / iterations;
  }

  public BooleanSupplier detectSpike() {
    if (iterations > 10) {
      if (lastCurrent > getAverageCurrent() * 2) {
        return () -> detectSpike(1);
      }
    }
    return () -> false;
  }

  private boolean detectSpike(int timesCalled) {
    if (timesCalled == 5) {
      return true;
    } else if (lastCurrent > getAverageCurrent() * 2) {
      return detectSpike(timesCalled + 1);
    } else {
      return false;
    }
  }
}
