package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
<<<<<<< HEAD:src/test/subsystems/ShooterTest.java
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Shooter.Shooter;
import org.junit.jupiter.api.BeforeEach;
import edu.wpi.first.wpilibj.Timer;
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

>>>>>>> 3e97e935276aa60a78ef3f17b6518738bda74cf6:src/test/java/frc/robot/subsystems/Shooter/ShooterTest.java
public class ShooterTest {
  Shooter shooter;


  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    shooter = new Shooter(() -> new Pose2d(), () -> new Pose2d(), () -> new ChassisSpeeds());
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    shooter.close(); // destroy our shooter object
  }
  /*
  @Test
  void testGetFlywheelPosition() {
    shooter.flywheelSim.setAngle(1);
    shooter.simulationPeriodic();
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

