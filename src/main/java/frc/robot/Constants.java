package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Map;

@Logged
public class Constants {
  public static final Time kDT = Seconds.of(0.02);
  public static final boolean tuningMode = true;
  public static final CANBus kCanBusBlinky = new CANBus("blinky");
  public static final CANBus kCanBusRio = new CANBus("rio");
  public static final double kSlowMoveRate = 4;
  // dont ask its the answer to everything
  private static final Pose3d kBlueHubPose =
      new Pose3d(Inches.of(182.11), Inches.of(158.84), Inches.of(72), new Rotation3d());
  private static final Pose3d kRedHubPose =
      new Pose3d(Inches.of(469.11), Inches.of(158.84), Inches.of(72), new Rotation3d());

  public static DriverStation.Alliance getAlliance() {
    if (DriverStation.getAlliance().isEmpty()) {
      return DriverStation.Alliance.Blue;
    }
    return DriverStation.getAlliance().get();
  }

  public static Pose3d getHubLocation() {
    if (getAlliance().equals(DriverStation.Alliance.Blue)) {
      return kBlueHubPose;
    }
    return kRedHubPose;
  }

  public static Pose2d getHubLocation2d() {
    return getHubLocation().toPose2d();
  }

  public static final Pose2d kBlueAllienceCorner1 = new Pose2d();
  public static final Pose2d kBlueAllienceCorner2 =
      new Pose2d(Inches.of(182.11), Inches.of(317.69), new Rotation2d());
  public static final Pose2d kRedAllienceCorner1 =
      new Pose2d(Inches.of(469.11), Inches.of(0), new Rotation2d());
  public static final Pose2d kRedAllienceCorner2 =
      new Pose2d(Inches.of(651.22), Inches.of(317.69), new Rotation2d());

  public enum Trench {
    LowerBlueTrench,
    UpperBlueTrench,
    LowerRedTrench,
    UpperRedTrench
  }

  public static final Map<Trench, Pose2d[]> trenchCorners =
      Map.ofEntries(
          Map.entry(
              Trench.LowerBlueTrench,
              new Pose2d[] {
                new Pose2d(Meters.of(4), Meters.of(0), new Rotation2d()),
                new Pose2d(Meters.of(5.25), Meters.of(1.3), new Rotation2d())
              }),
          Map.entry(
              Trench.UpperBlueTrench,
              new Pose2d[] {
                new Pose2d(Meters.of(4), Meters.of(6.8), new Rotation2d()),
                new Pose2d(Meters.of(5.25), Meters.of(8.1), new Rotation2d())
              }),
          Map.entry(
              Trench.LowerRedTrench,
              new Pose2d[] {
                new Pose2d(Meters.of(11.25), Meters.of(0), new Rotation2d()),
                new Pose2d(Meters.of(12.6), Meters.of(1.3), new Rotation2d())
              }),
          Map.entry(
              Trench.UpperRedTrench,
              new Pose2d[] {
                new Pose2d(Meters.of(11.25), Meters.of(6.8), new Rotation2d()),
                new Pose2d(Meters.of(12.6), Meters.of(8.1), new Rotation2d())
              }));

  public static Pose2d[] getTrenchCorners(Trench trench) {
    return trenchCorners.get(trench);
  }

  public static Pose2d getTrenchCenter(Trench trench) {
    Pose2d[] corners = getTrenchCorners(trench);
    return new Pose2d(
        (corners[0].getX() + corners[1].getX()) * 0.5,
        (corners[0].getY() + corners[1].getY()) * 0.5,
        new Rotation2d());
  }

  public static Pose2d[] getTrenchCornersVelocity(
      Trench trench, ChassisSpeeds chassisSpeeds, Pose2d drivePose) {
    Pose2d[] corners = getTrenchCorners(trench);
    ChassisSpeeds speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, drivePose.getRotation()).div(2);
    double xMult = 1;
    double YMult = 1;
    if (drivePose.getX() > getTrenchCenter(trench).getX()) {
      xMult = -xMult;
    }
    // if (driveState.Pose.getY() < getTrenchCenter(trench).getY()) {
    //   YMult = -YMult;
    // }

    return new Pose2d[] {
      (corners[0].plus(
          new Transform2d(
              -speeds.vxMetersPerSecond * xMult - 1,
              -Math.abs(speeds.vyMetersPerSecond) * YMult - 1,
              new Rotation2d()))),
      (corners[1].plus(
          new Transform2d(
              speeds.vxMetersPerSecond * xMult + 1,
              Math.abs(speeds.vyMetersPerSecond) * YMult + 1,
              new Rotation2d())))
    };
  }
}
