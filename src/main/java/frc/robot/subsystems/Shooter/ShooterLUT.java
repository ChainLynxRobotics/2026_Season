package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotContainer;
import java.util.Optional;

public class ShooterLUT {
  public record ShooterSetpoint(LinearVelocity flywheelSurfaceSpeed, Angle rotation) {}

  public record SOTMSetpoint(
      ShooterSetpoint shooterSetpoint, Rotation2d robotRotation, Pose2d iteratedPose) {}

  public static ShooterSetpoint getSpeedAndRotation(Distance distance) {
    return new ShooterSetpoint(
        MetersPerSecond.of(kShooterSpeedMap.get(distance.in(Meters))),
        Degrees.of(kShooterAngleMap.get(distance.in(Meters))));
  }

  private static final double kMaxIterations = 5;
  private static final Distance kRecursionTarget = Meters.of(0.01);

  public static Optional<SOTMSetpoint> generateShootOnTheMoveSetpoint(
      Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    var lastPose = new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, new Rotation2d());
    var iteratedPose = robotPose;
    var distance = Shooter.getDistance(robotPose);
    var timeOfFlight = kShooterTOFMap.get(distance.in(Meters));
    var iterations = 0;
    while (iterations < kMaxIterations) {
      lastPose = iteratedPose;
      iteratedPose =
          new Pose2d(
              robotPose.getX() + robotSpeeds.vxMetersPerSecond * timeOfFlight,
              robotPose.getY() + robotSpeeds.vyMetersPerSecond * timeOfFlight,
              robotPose.getRotation());
      distance = Shooter.getDistance(iteratedPose);
      timeOfFlight = kShooterTOFMap.get(distance.in(Meters));
      if (iteratedPose.getTranslation().getDistance(lastPose.getTranslation())
          <= kRecursionTarget.in(Meters)) {
        return Optional.of(
            new SOTMSetpoint(
                getSpeedAndRotation(distance),
                RobotContainer.getAngleToHub(iteratedPose),
                iteratedPose));
      }
      iterations += 1;
    }
    System.out.println("Recursion didnt converge");
    System.out.println("Distance at end of recursion: " + distance.toString());
    return Optional.empty();
  }

  private static final InterpolatingDoubleTreeMap kShooterSpeedMap = generateSpeedMap();
  private static final InterpolatingDoubleTreeMap kShooterAngleMap = generateAngleMap();
  private static final InterpolatingDoubleTreeMap kShooterTOFMap = generateTOFMap();

  private static InterpolatingDoubleTreeMap generateSpeedMap() {
    var map = new InterpolatingDoubleTreeMap();
    if (RobotBase.isReal()) {
      map.put(1.8546, 60.0);
      map.put(2.221275, 65.0);
      map.put(2.647, 65.0);
      map.put(2.934, 67.5);
      map.put(3.333, 70.0);
      map.put(3.70912, 75.0);
      map.put(4.1389, 77.5);
      map.put(4.5181, 80.0);
    } else {
      map.put(1.25, 25.2);
      map.put(1.5, 26.5);
      map.put(1.7, 26.8);
      map.put(2.0, 27.0);
      map.put(2.5, 27.5);
      map.put(3.0, 28.1);
      map.put(3.5, 28.5);
      map.put(4.0, 29.2);
      map.put(4.5, 29.9);
      map.put(5.0, 30.4);
      map.put(5.5, 30.8);
      map.put(6.0, 31.5);
    }
    return map;
  }

  private static InterpolatingDoubleTreeMap generateAngleMap() {
    var map = new InterpolatingDoubleTreeMap();
    if (RobotBase.isReal()) {
      map.put(1.8546, 25.0);
      map.put(2.221275, 25.0);
      map.put(2.647, 27.5);
      map.put(2.934, 27.5);
      map.put(3.333, 27.5);
      map.put(3.70912, 27.5);
      map.put(4.1389, 30.0);
      map.put(4.5181, 30.0);
    } else {
      map.put(1.5, 8.56);
      map.put(2.0, 11.32);
      map.put(2.5, 13.63);
      map.put(3.0, 15.76);
      map.put(3.5, 18.04);
      map.put(4.0, 20.18);
      map.put(4.5, 22.24);
      map.put(5.0, 24.27);
      map.put(5.5, 26.23);
      map.put(6.0, 28.14);
    }
    return map;
  }

  private static InterpolatingDoubleTreeMap generateTOFMap() {
    var map = new InterpolatingDoubleTreeMap();
    if (RobotBase.isReal()) {

    } else {
      map.put(1.5, 1.1833);
      map.put(2.0, 1.16);
      map.put(2.5, 1.33);
      map.put(3.0, 1.216);
      map.put(3.5, 1.26);
      map.put(4.0, 1.33);
      map.put(4.5, 1.33);
      map.put(5.0, 1.3);
      map.put(5.5, 1.33);
      map.put(6.0, 0.7);
    }
    return map;
  }
}
