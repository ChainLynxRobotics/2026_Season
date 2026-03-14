package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import org.junit.jupiter.api.Test;

public class SpikeDetectorTest {
  @Test
  void testSpikeAbove() {
    var detector = new SpikeDetector(0.25, 0.04, 2, false);
    for (int i = 0; i < 10; i++) {
      Timer.delay(0.02);
      detector.update(10);
    }
    Timer.delay(0.02);
    detector.update(15);
    Timer.delay(0.02);
    assert (detector.update(15));
  }

  @Test
  void testSpikeBelow() {
    var detector = new SpikeDetector(0.25, 0.04, 2, true);
    for (int i = 0; i < 10; i++) {
      Timer.delay(0.02);
      detector.update(10);
    }
    Timer.delay(0.02);
    detector.update(5);
    Timer.delay(0.02);
    assert (detector.update(5));
  }
}
