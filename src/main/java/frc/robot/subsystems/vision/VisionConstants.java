package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

public class VisionConstants {
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double kAmbiguityTolerance = 0.25;
  public static final double kDistanceTolerance = 2.5;
  public static final double kMaxAngleTolerance = 70;
  // TODO: kMaxAngleTolerance is not used
  public static final Distance kFieldHeight = Meters.of(8.07); // y
  public static final Distance kFieldWidth = Meters.of(16.54); // x

  public static final List<Transform3d> kCameraOffsets =
      List.of(
          new Transform3d(0.268, 0.19684, 0.5715, new Rotation3d(0, 0, 0))
          /*new Transform3d(0, -0.4347972, 0.54864, new Rotation3d(0, 0, Math.PI / 2))*/ );

  public static Pose3d getHubLocation() {
    if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
      return new Pose3d(Inches.of(469.11), Inches.of(158.84), Inches.of(72), new Rotation3d());
    }
    return new Pose3d(Inches.of(182.11), Inches.of(158.84), Inches.of(72), new Rotation3d());
  }

  public static final Pose3d kHubLocation = getHubLocation();
  public static final Matrix<N3, N1> kBaseDeviation = VecBuilder.fill(1.5, 1.5, 10);
}
