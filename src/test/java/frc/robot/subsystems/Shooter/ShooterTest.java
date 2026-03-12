package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.RobotMath;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShooterTest {
  Shooter shooter;
  private TalonFX flywheelMotor = new TalonFX(kFlywheelCANId);
  private TalonFX hoodMotor = new TalonFX(kHoodCANId);

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    shooter =
        new Shooter(
            () -> new Pose2d(),
            () -> new Pose2d(),
            () -> new ChassisSpeeds(),
            flywheelMotor,
            hoodMotor,
            () -> (false),() -> new CommandXboxController(1));
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Timer.delay(1);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    shooter.close();
    // destroy our shooter object
  }

  // @Test
  // void testCreateHoodSetpoint() {
  //   shooter.setHoodAngleInternal(Radians.of(1));
  //   for (int i = 0; i < 100; ++i) {
  //     shooter.simulationPeriodic();
  //     System.out.println(shooter.getHoodPosition().in(Radians));
  //     Timer.delay(0.02);
  //     if (RobotMath.isWithinTolerance(shooter.getHoodPosition(), Radians.of(1), Radians.of(1e-2))
  //         && i > 3) {
  //       System.out.println(i + " ticks");
  //       break;
  //     }
  //   }
  //   assertEquals(shooter.getHoodPosition().in(Radians), 1, 1e-2);
  // }

  @Test
  void testGetHoodPosition() {
    shooter.hoodSim.setAngle(1);
    for (int i = 0; i < 100; ++i) {
      shooter.simulationPeriodic();
      System.out.println(shooter.getHoodPosition().in(Radians));
      Timer.delay(0.02);
      if (RobotMath.isWithinTolerance(shooter.getHoodPosition(), Radians.of(1), Radians.of(1e-2))
          && i > 3) {
        System.out.println(i + " ticks");
        break;
      }
    }
    assertEquals(shooter.getHoodPosition().in(Radians), 1, 1e-2);
  }

  @Test
  void testGetFlywheelPosition() {
    shooter.flywheelSim.setAngle(1);
    for (int i = 0; i < 100; ++i) {
      shooter.simulationPeriodic();
      System.out.println(shooter.getFlywheelPosition());
      Timer.delay(0.02);
      if (RobotMath.isWithinTolerance(
              shooter.getFlywheelPosition(), Radians.of(1), Radians.of(1e-3))
          && i > 3) {
        System.out.println(i + " ticks");
        break;
      }
    }
    assertEquals(shooter.getFlywheelPosition().in(Radians), 1, 1e-3);
  }

  @Test
  void TestFlywheelRotation() {
    shooter.setFlywheelVelocityInternal(RadiansPerSecond.of(20));
    for (int i = 0; i < 100; ++i) {
      shooter.simulationPeriodic();
      System.out.println(shooter.getFlywheelVelocity());
      Timer.delay(0.02);
      if (RobotMath.isWithinTolerance(
              shooter.getFlywheelVelocity(),
              RadiansPerSecond.of(20), // shooter.getFlywheelSetpoint()),
              RadiansPerSecond.of(2))
          && i > 3) {
        System.out.println(i + " ticks");
        break;
      }
    }
    assertTrue(
        RobotMath.isWithinTolerance(
            shooter.getFlywheelVelocity(),
            RadiansPerSecond.of(20), // shooter.getFlywheelSetpoint()),
            RadiansPerSecond.of(2)));
  }
}
