package frc.robot.utils;

import edu.wpi.first.math.filter.Debouncer;

public class SpikeDetector {
  private final double averageAccumulation;
  private double rollingAverage = 0;
  private final Debouncer debouncer;
  private final double spikeAmount;
  private final boolean below;

  public SpikeDetector(
      double averageAccumulation, double debounceTime, double spikeAmount, boolean below) {
    this.averageAccumulation = averageAccumulation;
    this.debouncer = new Debouncer(debounceTime);
    this.spikeAmount = spikeAmount;
    this.below = below;
  }

  public boolean update(double value) {
    rollingAverage = rollingAverage * (1 - averageAccumulation) + (value * averageAccumulation);
    boolean spikedNoDebounce =
        below ? value < rollingAverage - spikeAmount : value > rollingAverage + spikeAmount;
    return debouncer.calculate(spikedNoDebounce);
  }
}
