package frc.robot.test;

import static frc.robot.subsystems.Intake.IntakeConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

public class IntakeTest {
  IntakeSubsystem intake;
  TalonFX spinMotor;
  TalonFXSimState simSpinMotor;
  TalonFX heightMotor;
  TalonFXSimState simHeightMotor;

  void setup() {
    spinMotor = new TalonFX(0);
    simSpinMotor = new TalonFXSimState(spinMotor);
    heightMotor = new TalonFX(0);
    simHeightMotor = new TalonFXSimState(heightMotor);
    intake = new IntakeSubsystem(spinMotor, heightMotor);
  }

  @Test
  void testIntakeDown() {
    /*
    simHeightMotor.setSupplyVoltage(RobotController.getBatteryVoltage());

    climber.setFullPower();
    climber.setStateSetpoint(ClimberState.TOP);

    for (int i = 0; i < 100; i += 1) {
      Timer.delay(0.020);
      DCMotorSim.setInputVoltage(simHeightMotor.getMotorVoltage());
      System.out.println(simHeightMotor.getMotorVoltage());
      DCMotorSim.update(0.020);

      simHeightMotor.setRawRotorPosition(DCMotorSim.getAngularPosition().times(kGearRatio));
      simHeightMotor.setRotorVelocity(DCMotorSim.getAngularVelocity().times(kGearRatio));
      // System.out.println(climber.atSetpoint() ? "yes" : "no");
      System.out.println(climber.getPosition());
      if (climber.atSetpoint()) {
        System.out.println(i + " ticks");
        break;
      }
    }
    assertTrue(climber.atSetpoint());
    intake.setHeight(IntakeHeightState.LOW);
    assertEquals(IntakeHeightState.LOW.getAngle(), intake.getHeightPosition());
    */
  }

  @Test
  void testIntakeVelocity() {
    intake.spin();
    assertEquals(kGoalIntakeSpinVelocity, intake.getSpinVelocity());
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    spinMotor.close();
    heightMotor.close();
    intake.close();
  }
}
