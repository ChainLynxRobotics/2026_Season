import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.Shooter.Shooter;
import org.junit.jupiter.api.BeforeEach;

public class ShooterTest {
  Shooter shooter;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    shooter = new Shooter();
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    shooter.close(); // destroy our intake object
  }

  @Test
  void testGetFlywheelPosition() {
    shooter.flywheelSim.setAngle(1);
    assertEquals(shooter.getFlywheelPosition().in(Radians), 1 * kHoodGearRatio, 1e-6);
  }
}
