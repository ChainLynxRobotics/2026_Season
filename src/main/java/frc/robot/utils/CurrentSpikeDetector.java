package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CurrentSpikeDetector {
  private Current totalCurrent;
  private Current lastCurrent;
  private int iterations = 0;

  private Debouncer debouncer = new Debouncer(0.2);

  private Supplier<Current> currentSupplier;

  TalonFX talonFXMotor;

  public CurrentSpikeDetector(Supplier<Current> currentSupplier) {
    this.currentSupplier = currentSupplier;
  }

  public void update() {
    iterations++;
    lastCurrent = currentSupplier.get();
    totalCurrent = totalCurrent.plus(lastCurrent);
  }

  public Current getAverageCurrent() {
    return totalCurrent.div(iterations);
  }

  public BooleanSupplier getSpikeSupplier() {
    return this::detectSpike;
  }

  private boolean detectSpike() {
    update();
    return debouncer.calculate(lastCurrent.gt(getAverageCurrent().times(2)) && iterations > 10);
  }
}
