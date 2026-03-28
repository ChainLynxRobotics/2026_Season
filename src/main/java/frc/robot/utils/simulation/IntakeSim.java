package frc.robot.utils.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

@Logged
public class IntakeSim {
  public IntakeSimulation intakeSimulation;
  private Runnable shootFuel;
  public int timeLastBall = 0;

  public IntakeSim(SwerveDriveSimulation driveTrain, Runnable shootFuel) {
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveTrain,
            Meters.of(0.629611),
            Meters.of(0.20608),
            IntakeSimulation.IntakeSide.FRONT,
            55);
    intakeSimulation.startIntake();

    this.shootFuel = shootFuel;
  }

  public void setRunning(boolean runIntake) {
    if (runIntake) intakeSimulation.startIntake();
    else intakeSimulation.stopIntake();
  }

  public void shootGamePiece() {
    if (timeLastBall >= 15) {
      if (intakeSimulation.obtainGamePieceFromIntake()) {
        shootFuel.run();
      }
      timeLastBall = 0;
    } else {
      timeLastBall += 1;
    }
  }
}
