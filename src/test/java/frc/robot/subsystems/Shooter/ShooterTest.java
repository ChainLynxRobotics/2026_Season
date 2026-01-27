package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.Shooter.Shooter;
import org.junit.jupiter.api.BeforeEach;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
}
