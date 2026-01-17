package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.*;
import java.util.ArrayList;
import java.util.List;

public class VisionConstants {
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double kAmbiguityTolerance = 0.25;
  public static final double kDistanceTolerance = 2.5;
  public static final double kMaxAngleTolerance = 70;
  public static final double kFieldHeight = 8.05; // y
  public static final double kFieldWidth = 17.55; // x

  public static List<Transform3d> kCameraOffsets =
      new ArrayList<>(
          List.of(
              new Transform3d(0, 0.4347972, 0.54864, new Rotation3d(0, 0, 0)),
              new Transform3d(0, -0.4347972, 0.54864, new Rotation3d(0, 0, Math.PI / 2))));

  public static Matrix<N3, N1> kBaseDeviation = VecBuilder.fill(0.1, 0.1, 0.5);
}
