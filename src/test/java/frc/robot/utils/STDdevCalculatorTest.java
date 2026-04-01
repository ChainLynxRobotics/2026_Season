package frc.robot.utils;

import static frc.robot.utils.STDevCalculator.calculateSTDevs;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class STDdevCalculatorTest {
  @Test
  public void calculatorTest() {
    assertEquals(calculateSTDevs(new ArrayList<>()), 0);
    assertEquals(calculateSTDevs(List.of(1.0)), 0);
    assertEquals(calculateSTDevs(List.of(1.0, 1.0, 1.0, 1.0, 1.0)), 0);
    assertEquals(
        calculateSTDevs(List.of(24.0, 72.0, 12.0, 13.0, 92.0, 62.0, 34.0, 8.0)),
        29.6645305878923,
        0.000001);
  }
}
