import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants.ClimberState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ClimberTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  Climber climber;
  TalonFXSimState simulatedMotor;
  TalonFX motor;
  DCMotorSim DCMotorSim;
  ElevatorSim elevatorSim;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    motor = new TalonFX(1, new CANBus("blinky"));
    simulatedMotor = new TalonFXSimState(motor); // create our sim Talon
    climber = new Climber(motor); // create our climber

    /* create the simulated DC motor */
    DCMotor gearbox = DCMotor.getKrakenX60Foc(1);

    DCMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, kGearRatio), gearbox);

    elevatorSim =
        new ElevatorSim(gearbox, kGearRatio, 55, Inches.of(0.5).in(Meters), 0, 120, true, 0);

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Timer.delay(0.100);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    motor.close();
    climber.close(); // destroy our climber object
  }

  /*
  @Test
  void simMotorRawTest() {
    simulatedMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

    climber.setFullPower();
    climber.setStateSetpoint(ClimberState.TOP);

    for (int i = 0; i < 100; ++i) {
      Timer.delay(0.020);
      DCMotorSim.setInputVoltage(simulatedMotor.getMotorVoltage());
      System.out.println(simulatedMotor.getMotorVoltage());
      DCMotorSim.update(0.020);

      simulatedMotor.setRawRotorPosition(DCMotorSim.getAngularPosition().times(kGearRatio));
      simulatedMotor.setRotorVelocity(DCMotorSim.getAngularVelocity().times(kGearRatio));
      // System.out.println(climber.atSetpoint() ? "yes" : "no");
      System.out.println(climber.getPosition());
      if (climber.atSetpoint()) {
        System.out.println(i + " ticks");
        break;
      }
    }
    assertTrue(climber.atSetpoint());
  }
  */

  @Test
  void simMotorRawTest2() {
    simulatedMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

    climber.setFullPower();
    climber.setStateSetpoint(ClimberState.TOP);

    for (int i = 0; i < 100; ++i) {
      Timer.delay(0.020);
      System.out.println(simulatedMotor.getMotorVoltage());
      elevatorSim.setInputVoltage(simulatedMotor.getMotorVoltage());
      elevatorSim.update(0.020);

      simulatedMotor.setRawRotorPosition(elevatorSim.getPositionMeters());
      simulatedMotor.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond());
      System.out.println(climber.getPosition());
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
      if (climber.atSetpoint()) {
        System.out.println(i + " ticks");
        break;
      }
    }
    assertTrue(climber.atSetpoint());
  }
}
