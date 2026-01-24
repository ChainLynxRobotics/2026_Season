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
  TalonFX motor;
  TalonFXSimState simMotor;

  void setup() {
    motor = new TalonFX(0);
    simMotor = new TalonFXSimState(motor);
    intake = new IntakeSubsystem();
  }

  @Test
  void testIntakeUp() {
    intake.setHeight(IntakeState.HIGH);
  }

  @Test
  void testIntakeDown() {
    intake.setHeight(IntakeState.LOW);
    assertEquals(IntakeState.LOW.getAngle(), intake.getIntakeHeightPosition());
  }

  @Test
  void testIntakeVelocity() {
    intake.spin();
    assertEquals(kGoalIntakeSpinVelocity, intake.getIntakeSpinVelocity());
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    motor.close();
    intake.close();
  }
}
