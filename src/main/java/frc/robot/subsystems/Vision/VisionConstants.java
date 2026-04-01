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
import java.util.List;

public class VisionConstants {
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

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
              new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.of(90))),
          new Transform3d(
              Inches.of(-12.55),
              Inches.of(-8.969),
              Inches.of(9.766),
              new Rotation3d(Degrees.zero(), Degrees.of(-30), Degrees.of(210))));

  public static final Matrix<N3, N1> kBaseDeviation = VecBuilder.fill(1.5, 1.5, 10);
}
