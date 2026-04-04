package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import java.util.List;

public class VisionConstants {

  // SET TO FALSE BEFORE COMP — uses hallway.json tag layout instead of the competition field
  public static final boolean isHallwayField = false;

  private static AprilTagFieldLayout getFieldLayout() {
    if (isHallwayField) {
      try {
        return new AprilTagFieldLayout(
            Path.of(Filesystem.getDeployDirectory().getAbsolutePath(), "hallway.json"));
      } catch (Exception ex) {
        DriverStation.reportError(
            "Failed to load hallway.json, falling back to comp field", ex.getStackTrace());
        return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
      }
    } else {
      return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    }
  }

  public static final AprilTagFieldLayout kTagLayout = getFieldLayout();

  public static final boolean kKeepTrackOfSTDevs = true;

  public static final double kAmbiguityTolerance = 15;
  public static final double kDistanceTolerance = 5;
  public static final double kMaxAngleTolerance = 70;
  // TODO: kMaxAngleTolerance is not used
  public static final Distance kFieldHeight = Meters.of(8.07); // y
  public static final Distance kFieldWidth = Meters.of(16.54); // x

  public static final List<Transform3d> kCameraOffsets =
      List.of(
          new Transform3d(
              Inches.of(8.522),
              Inches.of(12.473),
              Inches.of(10.88),
              new Rotation3d(Degrees.zero(), Degrees.of(5), Degrees.of(90))),
          new Transform3d(
              Inches.of(-12.55),
              Inches.of(-8.969),
              Inches.of(9.766),
              new Rotation3d(Degrees.zero(), Degrees.of(5), Degrees.of(210))));

  public static final Matrix<N3, N1> kBaseDeviation = VecBuilder.fill(1.5, 1.5, 10);

  // Pinhole camera intrinsics — update with real calibration from PhotonVision coprocessor
  // Defaults match a 1280x800 camera with ~70 deg diagonal FOV
  public static final double kCameraFx = 1078;
  public static final double kCameraFy = 1078;
  public static final double kCameraCx = 640;
  public static final double kCameraCy = 400;

  public static final double kTagSizeMeters = 0.1651; // 6.5 inches
}
