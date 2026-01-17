package frc.robot.test;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.Intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeTest {
  IntakeSubsystem intake;

  void setup() {
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
}
