package frc.robot.utils;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.kFunnlingLocation;
import static frc.robot.subsystems.Shooter.ShooterConstants.kShooterLocation;
import static frc.robot.subsystems.Vision.VisionConstants.kFieldHeight;
import static frc.robot.subsystems.Vision.VisionConstants.kFieldWidth;
import static frc.robot.utils.RobotMath.getDistanceBetweenPoses;
import static frc.robot.utils.RobotMath.isPoseInSquare;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterLUT;

public class PointingUtil {

  public static Pose2d getShootingTarget(Pose2d robotPose) {
    var isInAllianceZone = false;
    if (getAlliance() == DriverStation.Alliance.Blue) {
      isInAllianceZone = isPoseInSquare(robotPose, kBlueAllienceCorner1, kBlueAllienceCorner2);
    } else {
      isInAllianceZone = isPoseInSquare(robotPose, kRedAllienceCorner1, kRedAllienceCorner2);
    }

    if (isInAllianceZone) return getHubLocation2d();

    var funnlingPoint1 =
        getAlliance() == Alliance.Blue
            ? kFunnlingLocation
            : new Pose2d(
                kFieldWidth.minus(kFunnlingLocation.getMeasureX()),
                kFunnlingLocation.getMeasureY(),
                new Rotation2d());
    var funnlingPoint2 =
        new Pose2d(
            funnlingPoint1.getMeasureX(),
            kFieldHeight.minus(funnlingPoint1.getMeasureY()),
            new Rotation2d());

    return getDistanceBetweenPoses(funnlingPoint1, robotPose)
            .lt(getDistanceBetweenPoses(funnlingPoint2, robotPose))
        ? funnlingPoint1
        : funnlingPoint2;
  }

  public static Rotation2d getAngleToPose(Pose2d robotPose, Pose2d targetPose) {
    return Shooter.getShooterPose(robotPose)
        .getTranslation()
        .minus(targetPose.getTranslation())
        .getAngle()
        .plus(kShooterLocation.getRotation());
  }

  private static Rotation2d lastRotation = new Rotation2d();

  public static Rotation2d getAngleToPoseTOF(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, Pose2d targetPose) {
    var setpoint = ShooterLUT.generateShootOnTheMoveSetpoint(robotPose, robotSpeeds, targetPose);
    if (setpoint.isEmpty()) {
      return lastRotation;
    }
    lastRotation = setpoint.get().robotRotation();
    return lastRotation;
  }

  public static AngularVelocity getTOFRotationalVelocityToTarget(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, Pose2d targetPose) {
    var dt = 0.01;
    var poseInDt =
        new Pose2d(
            robotPose.getX() + robotSpeeds.vxMetersPerSecond * dt,
            robotPose.getY() + robotSpeeds.vyMetersPerSecond * dt,
            new Rotation2d(
                robotPose.getRotation().getRadians() + robotSpeeds.omegaRadiansPerSecond * dt));
    return RotationsPerSecond.of(
        getAngleToPoseTOF(poseInDt, robotSpeeds, targetPose)
                .getMeasure()
                .minus(getAngleToPoseTOF(robotPose, robotSpeeds, targetPose).getMeasure())
                .in(Rotations)
            / dt);
  }

  public static Angle optimiseRotation(Rotation2d currentAngle, Rotation2d targetAngle) {
    var delta =
        MathUtil.angleModulus(targetAngle.getRadians())
            - MathUtil.angleModulus(currentAngle.getRadians());
    if (delta < -Math.PI) {
      return Radians.of(delta + 2 * Math.PI);
    } else if (delta > Math.PI) {
      return Radians.of(delta - 2 * Math.PI);
    } else {
      return Radians.of(delta);
    }
  }
}
