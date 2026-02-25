package frc.robot.utils;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.getHubLocation2d;
import static frc.robot.subsystems.Shooter.ShooterConstants.kShooterLocation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterLUT;

public class PointingUtil {

  public static Rotation2d getAngleToHub(Pose2d robotPose) {
    return Shooter.getShooterPose(robotPose)
        .getTranslation()
        .minus(getHubLocation2d().getTranslation())
        .getAngle()
        .plus(kShooterLocation.getRotation());
  }

  private static Rotation2d lastRotation = new Rotation2d();

  public static Rotation2d getAngleToHubTOF(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    var setpoint = ShooterLUT.generateShootOnTheMoveSetpoint(robotPose, robotSpeeds);
    if (setpoint.isEmpty()) {
      return lastRotation;
    }
    lastRotation = setpoint.get().robotRotation();
    return lastRotation;
  }

  public static AngularVelocity getTOFRotationalVelocityToHub(
      Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    var dt = 0.01;
    var poseInDt =
        new Pose2d(
            robotPose.getX() + robotSpeeds.vxMetersPerSecond * dt,
            robotPose.getY() + robotSpeeds.vyMetersPerSecond * dt,
            new Rotation2d(
                robotPose.getRotation().getRadians() + robotSpeeds.omegaRadiansPerSecond * dt));
    return RotationsPerSecond.of(
        getAngleToHubTOF(poseInDt, robotSpeeds)
                .getMeasure()
                .minus(getAngleToHubTOF(robotPose, robotSpeeds).getMeasure())
                .in(Rotations)
            / dt);
  }

  public static AngularVelocity getRotationalVelocityToHub(
      Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    var dt = 0.01;
    var poseInDt =
        new Pose2d(
            robotPose.getX() + robotSpeeds.vxMetersPerSecond * dt,
            robotPose.getY() + robotSpeeds.vyMetersPerSecond * dt,
            new Rotation2d(
                robotPose.getRotation().getRadians() + robotSpeeds.omegaRadiansPerSecond * dt));
    return RotationsPerSecond.of(
        getAngleToHub(poseInDt)
                .getMeasure()
                .minus(getAngleToHub(robotPose).getMeasure())
                .in(Rotations)
            / dt);
  }
}
