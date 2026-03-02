package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.PointingUtil;
import java.util.Optional;

public class ShooterLUT {
  public record ShooterSetpoint(AngularVelocity flywheelVelocity, Angle rotation) {}

  public record SOTMSetpoint(
      ShooterSetpoint shooterSetpoint, Rotation2d robotRotation, Pose2d iteratedPose) {}

  public static ShooterSetpoint getSpeedAndRotation(Distance distance) {
    return new ShooterSetpoint(
        RotationsPerSecond.of(kShooterSpeedMap.get(distance.in(Meters))),
        Degrees.of(kShooterAngleMap.get(distance.in(Meters))));
  }

  private static final double kMaxIterations = 15;
  private static final Distance kRecursionTarget = Meters.of(0.01);

  public static Optional<SOTMSetpoint> generateShootOnTheMoveSetpoint(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, Pose2d targetPose) {
    var lastPose = new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, new Rotation2d());
    var iteratedPose = robotPose;
    var distance = Shooter.getDistanceToPose(robotPose, targetPose);
    var timeOfFlight = kShooterTOFMap.get(distance.in(Meters));
    var iterations = 0;
    while (iterations < kMaxIterations) {
      lastPose = iteratedPose;
      iteratedPose =
          new Pose2d(
              robotPose.getX() + robotSpeeds.vxMetersPerSecond * timeOfFlight,
              robotPose.getY() + robotSpeeds.vyMetersPerSecond * timeOfFlight,
              PointingUtil.getAngleToPose(iteratedPose, targetPose));
      distance = Shooter.getDistanceToPose(iteratedPose, targetPose);
      timeOfFlight = kShooterTOFMap.get(distance.in(Meters));
      if (iteratedPose.getTranslation().getDistance(lastPose.getTranslation())
          <= kRecursionTarget.in(Meters)) {
        return Optional.of(
            new SOTMSetpoint(
                getSpeedAndRotation(distance),
                PointingUtil.getAngleToPose(iteratedPose, targetPose),
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
      map.put(1.041, 60.0);
      map.put(1.92, 65.0);
      map.put(2.31, 65.0);
      map.put(3.25, 72.0);
      map.put(4.17, 78.0);
      map.put(5.71, 92.0);
      map.put(7.37, 103.0);
    } else {
      map.put(1.041, 60.0);
      map.put(1.92, 65.0);
      map.put(2.31, 65.0);
      map.put(3.25, 72.0);
      map.put(4.17, 78.0);
      map.put(5.71, 92.0);
      map.put(7.37, 103.0);
    }
    return map;
  }

  private static InterpolatingDoubleTreeMap generateAngleMap() {
    var map = new InterpolatingDoubleTreeMap();
    if (RobotBase.isReal()) {
      map.put(1.041, 18.0);
      map.put(1.92, 23.0);
      map.put(2.31, 24.0);
      map.put(3.25, 27.5);
      map.put(4.17, 29.0);
      map.put(5.71, 32.0);
      map.put(7.37, 43.0);
    } else {
      map.put(1.041, 18.0);
      map.put(1.92, 23.0);
      map.put(2.31, 24.0);
      map.put(3.25, 27.5);
      map.put(4.17, 29.0);
      map.put(5.71, 32.0);
      map.put(7.37, 43.0);
    }
    return map;
  }

  private static InterpolatingDoubleTreeMap generateTOFMap() {
    var map = new InterpolatingDoubleTreeMap();
    if (RobotBase.isReal()) {
      map.put(1.041, 1.516);
      map.put(1.92, 1.651);
      map.put(2.31, 1.996);
      map.put(3.25, 1.991);
      map.put(4.17, 1.486);
      map.put(5.71, 1.64);
      map.put(7.37, 1.59);
    } else {
      map.put(1.041, 1.516);
      map.put(1.92, 1.651);
      map.put(2.31, 1.996);
      map.put(3.25, 1.991);
      map.put(4.17, 1.486);
      map.put(5.71, 1.64);
      map.put(7.37, 1.59);
    }
    return map;
  }
}
