package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.RobotMath;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShooterTest {
  Shooter shooter;
  private TalonFX flywheelMotor=
      new TalonFX(kFlywheelCANId);
  private DCMotorSim flywheelSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              kFlywheelMotor, kFlywheelMOI.in(KilogramSquareMeters), kFlywheelGearRatio),
          kFlywheelMotor);
  ;
  private TalonFXSimState flywheelMotorSim;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    shooter =
        new Shooter(
            () -> new Pose2d(), () -> new Pose2d(), () -> new ChassisSpeeds(), flywheelMotor);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    shooter.close();
    // destroy our shooter object
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
    flywheelSim.setInputVoltage(RobotController.getBatteryVoltage());
    // shooter.setFlywheelVelocity(RadiansPerSecond.of(20));
    shooter.flywheelVoltageDrive(Volts.of(4));
    for (int i = 0; i < 100; ++i) {
      Timer.delay(0.020);
      flywheelSim.setInputVoltage(shooter.flywheelMotorSim.getMotorVoltage());
      System.out.println(shooter.flywheelMotorSim.getMotorVoltage());
      flywheelSim.update(0.020);

      shooter.flywheelMotorSim.setRawRotorPosition(
          shooter.flywheelSim.getAngularPosition().times(kFlywheelGearRatio));
      shooter.flywheelMotorSim.setRotorVelocity(
          shooter.flywheelSim.getAngularVelocity().times(kFlywheelGearRatio));
      // System.out.println(climber.atSetpoint() ? "yes" : "no");
      System.out.println(flywheelSim.getAngularVelocity());
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
