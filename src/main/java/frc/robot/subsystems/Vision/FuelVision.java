package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Vision.FuelVisionConstants.*;
import static frc.robot.subsystems.Vision.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class FuelVision extends SubsystemBase {

  private final PhotonCamera camera;
  private final Supplier<Pose2d> getRobotPose;

  private final List<FuelEntry> fuelMap = new ArrayList<>();

  private record FuelEntry(Translation2d position, double timestampSecs) {}

  public FuelVision(Supplier<Pose2d> getRobotPose) {
    this.camera = new PhotonCamera(kCameraName);
    this.getRobotPose = getRobotPose;
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      Pose2d robotPose = getRobotPose.get();
      double headingDeg = robotPose.getRotation().getDegrees();
      for (Pose3d piece : SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel")) {
        Translation2d fieldXY = piece.getTranslation().toTranslation2d();

        // Distance filter
        if (fieldXY.getDistance(robotPose.getTranslation()) > kMaxDetectionDistanceMeters) continue;

        // Horizontal FOV: bearing to piece relative to robot heading (camera faces forward)
        double bearingDeg =
            Math.toDegrees(
                Math.atan2(fieldXY.getY() - robotPose.getY(), fieldXY.getX() - robotPose.getX()));
        double relAngle = ((bearingDeg - headingDeg) % 360 + 360) % 360;
        if (relAngle > 180) relAngle = 360 - relAngle;
        if (relAngle > kHorizHalfFOVDegrees) continue;

        mergeIntoMap(fieldXY);
      }
      ageOutMap();
      return;
    }

    Pose2d robotPose = getRobotPose.get();

    for (var result : camera.getAllUnreadResults()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        // Airborne filter: skip if pitch is above the horizon threshold
        if (target.getPitch() > kMaxPitchDegrees) continue;

        // Floor projection: find field-frame XY where the bearing ray hits z = kFuelRadius
        Pose3d cameraPose = new Pose3d(robotPose).transformBy(kRobotToCamera);
        double yawRad = Math.toRadians(target.getYaw());
        double pitchRad = Math.toRadians(target.getPitch());

        // Unit bearing vector in camera NWU frame (+X forward, +Y left, +Z up)
        Translation3d dirCamera =
            new Translation3d(
                Math.cos(pitchRad) * Math.cos(yawRad),
                Math.cos(pitchRad) * Math.sin(yawRad),
                Math.sin(pitchRad));

        Translation3d dirField = dirCamera.rotateBy(cameraPose.getRotation());

        // Ray must point downward in field frame to intersect the floor
        double dirFieldZ = dirField.getZ();
        if (dirFieldZ >= 0) continue;

        double t = (kFuelRadiusMeters - cameraPose.getZ()) / dirFieldZ;
        if (t <= 0) continue;

        Translation2d fieldXY =
            new Translation2d(
                cameraPose.getX() + t * dirField.getX(), cameraPose.getY() + t * dirField.getY());

        // Distance filter
        if (fieldXY.getDistance(robotPose.getTranslation()) > kMaxDetectionDistanceMeters) continue;

        // Field bounds filter
        if (fieldXY.getX() < 0
            || fieldXY.getX() > kFieldWidth.in(Meters)
            || fieldXY.getY() < 0
            || fieldXY.getY() > kFieldHeight.in(Meters)) continue;

        mergeIntoMap(fieldXY);
      }
    }

    ageOutMap();
  }

  private void mergeIntoMap(Translation2d fieldXY) {
    for (int i = 0; i < fuelMap.size(); i++) {
      if (fuelMap.get(i).position().getDistance(fieldXY) < kMergeRadiusMeters) {
        // Blend toward new reading, refresh timestamp
        double blendedX = fuelMap.get(i).position().getX() * 0.7 + fieldXY.getX() * 0.3;
        double blendedY = fuelMap.get(i).position().getY() * 0.7 + fieldXY.getY() * 0.3;
        fuelMap.set(
            i, new FuelEntry(new Translation2d(blendedX, blendedY), Timer.getFPGATimestamp()));
        return;
      }
    }
    fuelMap.add(new FuelEntry(fieldXY, Timer.getFPGATimestamp()));
  }

  private void ageOutMap() {
    double now = Timer.getFPGATimestamp();
    fuelMap.removeIf(e -> now - e.timestampSecs() > kMapAgeSecs);
  }

  // ---- Public API ----

  /** All currently tracked fuel positions. */
  @NotLogged
  public List<Translation2d> getFuelMap() {
    return fuelMap.stream().map(FuelEntry::position).toList();
  }

  /** Nearest fuel to the given pose; empty if map has no entries. */
  public Optional<Translation2d> getNearestFuel(Pose2d robotPose) {
    return fuelMap.stream()
        .map(FuelEntry::position)
        .min(Comparator.comparingDouble(p -> p.getDistance(robotPose.getTranslation())));
  }

  /** Removes map entries near the given point (call after intake contact). */
  public void removeFuelNear(Translation2d point) {
    fuelMap.removeIf(e -> e.position().getDistance(point) < kMergeRadiusMeters);
  }

  /** Wipes the entire map (e.g., at auto start). */
  public void clearFuelMap() {
    fuelMap.clear();
  }

  // ---- Epilogue logging ----

  public Pose2d[] logFuelMap() {
    return fuelMap.stream()
        .map(e -> new Pose2d(e.position(), new Rotation2d()))
        .toArray(Pose2d[]::new);
  }

  public Pose3d[] logFuelMap3d() {
    return fuelMap.stream()
        .map(
            e ->
                new Pose3d(
                    e.position().getX(), e.position().getY(), kFuelRadiusMeters, new Rotation3d()))
        .toArray(Pose3d[]::new);
  }

  public int logFuelCount() {
    return fuelMap.size();
  }

  public double logNearestFuelMeters() {
    Translation2d robotXY = getRobotPose.get().getTranslation();
    return fuelMap.stream()
        .mapToDouble(e -> e.position().getDistance(robotXY))
        .min()
        .orElse(Double.NaN);
  }
}
