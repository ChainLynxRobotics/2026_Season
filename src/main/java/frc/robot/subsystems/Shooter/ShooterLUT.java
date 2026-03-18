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
  public record ShooterSetpoint(LinearVelocity flywheelSurfaceSpeed, Angle rotation) {}

  public record SOTMSetpoint(
      ShooterSetpoint shooterSetpoint, Rotation2d robotRotation, Pose2d iteratedPose) {}

  public static ShooterSetpoint getSpeedAndRotation(Distance distance) {
    return new ShooterSetpoint(
        MetersPerSecond.of(kShooterSpeedMap.get(distance.in(Meters))),
        Degrees.of(kShooterAngleMap.get(distance.in(Meters))));
  }

  private static final double kMaxIterations = 15;
  private static final Distance kRecursionTarget = Meters.of(0.01);
  private static Optional<SOTMSetpoint> cachedSetpoint = Optional.empty();
  private static Pose2d cachedRobotPose =
      new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, new Rotation2d(Double.MAX_VALUE));
  private static ChassisSpeeds cachedRobotSpeeds =
      new ChassisSpeeds(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private static Pose2d cachedTargetPose =
      new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, new Rotation2d(Double.MAX_VALUE));

  public static Optional<SOTMSetpoint> generateShootOnTheMoveSetpoint(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, Pose2d targetPose) {
    if (cachedRobotPose.equals(robotPose)
        && cachedRobotSpeeds.equals(robotSpeeds)
        && cachedTargetPose.equals(targetPose)) {
      return cachedSetpoint;
    }
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
        var setpoint =
            Optional.of(
                new SOTMSetpoint(
                    getSpeedAndRotation(distance),
                    PointingUtil.getAngleToPose(iteratedPose, targetPose),
                    iteratedPose));
        cachedSetpoint = setpoint;
        cachedRobotPose = robotPose;
        cachedRobotSpeeds = robotSpeeds;
        cachedTargetPose = targetPose;
        return setpoint;
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
      // Joe: These values are supposed to be flywheel surface speed in m/s, but 60-80 seems really
      // high for m/s — are these actually RPS from the old shooter? Might need retuning.
      map.put(1.01, 52.0);
      map.put(1.9050, 50.0);
      map.put(2.7178, 57.0);
      map.put(3.511, 62.0);
      map.put(4.21, 72.0);
      map.put(5.2400, 80.0);
      map.put(6.114, 84.5);
      map.put(7.27, 93.0);
      map.put(11.938, 140.0);
      // map.put(1.041, 60.0);
      // map.put(1.92, 65.0);
      // map.put(2.31, 65.0);
      // map.put(3.25, 72.0);
      // map.put(4.17, 78.0);
      // map.put(5.71, 92.0);
      // map.put(7.37, 103.0);
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
      map.put(1.01, 10.0);
      map.put(1.9050, 18.0);
      map.put(2.7178, 20.0);
      map.put(3.511, 22.0);
      map.put(4.21, 23.0);
      map.put(5.2400, 24.0);
      map.put(6.114, 26.0);
      map.put(7.27, 27.0);
      map.put(11.938, 35.0);
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
      map.put(1.01, 1.349);
      // TODO: fix please! this video was not posted last night and needs to be adjusted with the acvtual data
      map.put(1.9050, 1.15);
      map.put(2.7178, 1.05875);
      map.put(3.511, 1.22);
      map.put(4.21, 1.417);
      map.put(5.2400, 1.5355);
      map.put(6.114, 1.599);
      map.put(7.27, 1.786);
      map.put(11.938, 2.0695);
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
