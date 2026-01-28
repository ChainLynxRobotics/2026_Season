import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Shooter.Shooter;
import org.junit.jupiter.api.BeforeEach;
import edu.wpi.first.wpilibj.Timer;
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
  /*
  @Test
  void testGetFlywheelPosition() {
    shooter.flywheelSim.setAngle(1);
    assertEquals(shooter.getFlywheelPosition().in(Radians), 1 * kHoodGearRatio, 1e-6);
  }
  */

  
  @Test
  void TestFlywheelRotation() {
    shooter.flywheelSim.setInputVoltage(RobotController.getBatteryVoltage());

    shooter.flywheelSim.setAngularVelocity(RadiansPerSecond.of(20));

    for (int i = 0; i < 100; ++i) {
      Timer.delay(0.020);
      DCMotorSim.setInputVoltage(simulatedMotor.getMotorVoltage());
      System.out.println(simulatedMotor.getMotorVoltage());
      DCMotorSim.update(0.020);

      simulatedMotor.setRawRotorPosition(DCMotorSim.getAngularPosition().times(kGearRatio));
      simulatedMotor.setRotorVelocity(DCMotorSim.getAngularVelocity().times(kGearRatio));
      // System.out.println(climber.atSetpoint() ? "yes" : "no");
      System.out.println(shooter.flywheelSim.getAngularVelocity());
      if (climber.atSetpoint()) {
        System.out.println(i + " ticks");
        break;
      }
    }
  }
  assertTrue(climber.atSetpoint());
}

