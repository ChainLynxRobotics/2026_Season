package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class FuelVisionConstants {
  public static final String kCameraName = "fuelCam";

  // TODO: measure and fill in the actual robot-to-camera offset after mounting.
  // X is forward, Y is left, Z is up from robot center; negative pitch tilts the camera down.
  // Currently placed at front-center of robot frame, 0.5m high, aimed 30° down.
  public static final Transform3d kRobotToCamera =
      new Transform3d(
          Inches.of(14.0),
          Meters.of(0.0),
          Meters.of(0.5),
          new Rotation3d(Degrees.zero(), Degrees.of(-30), Degrees.zero()));

  // Arducam OV9782 with 70°(H) M12 lens, 1280x800 sensor.
  // HFOV = 70° (from spec sheet). VFOV derived: 2*atan(tan(35°)*800/1280) ≈ 47.3°
  public static final double kHorizHalfFOVDegrees = 35.0; // half of 70° HFOV
  public static final double kVertHalfFOVDegrees = 23.7; // half of ~47.3° VFOV

  // Fuel ball radius in meters; used as the floor-plane intersection height (z = ball center).
  public static final double kFuelRadiusMeters = 0.0762;
  // Discard floor projections farther than this distance from the robot.
  public static final double kMaxDetectionDistanceMeters = 4.0;
  // Two detections within this radius are treated as the same game piece.
  public static final double kMergeRadiusMeters = 0.25;
  // Map entries not refreshed within this number of seconds are pruned.
  public static final double kMapAgeSecs = 1.0;
  // If a target's pitch exceeds this, the piece is likely airborne. Tune based on camera tilt.
  public static final double kMaxPitchDegrees = 5.0;
}
