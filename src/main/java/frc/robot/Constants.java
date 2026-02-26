package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {
  public static final Time kDT = Seconds.of(0.02);
  public static final boolean tuningMode = true;
  public static final CANBus kCanBusBlinky = new CANBus("blinky");
  public static final CANBus kCanBusRio = new CANBus("rio");

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
}
