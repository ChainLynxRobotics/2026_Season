package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Vision.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class Vision extends SubsystemBase {

  private List<CamAndEstimator> cameras = new ArrayList<>();

  private Consumer<VisionPose> updateDrivetrain;
  private Supplier<Pose2d> getSimPose;

  private VisionSystemSim visionSim;
  private SimCameraProperties cameraProp;

  public record CamAndEstimator(PhotonPoseEstimator estimator, PhotonCamera camera) {}

  public record VisionPose(Pose3d pose, double timestamp, Matrix<N3, N1> deviation) {}

  @NotLogged
  private VisionPose swervePose =
      new VisionPose(new Pose3d(), 0, new Matrix<>(N3.instance, N1.instance));

  public Vision(Consumer<VisionPose> updateDrivetrain, Supplier<Pose2d> getSimPose) {
    this.updateDrivetrain = updateDrivetrain;
    this.getSimPose = getSimPose;

    cameras.add(
        new CamAndEstimator(
            new PhotonPoseEstimator(kTagLayout, kCameraOffsets.get(1)),
            new PhotonCamera("aprilOne")));

    cameras.add(
        new CamAndEstimator(
            new PhotonPoseEstimator(kTagLayout, kCameraOffsets.get(0)),
            new PhotonCamera("aprilTwo")));

    if (!RobotBase.isReal()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(kTagLayout);
      cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
      cameraProp.setCalibError(0.25, 0.08);
      cameraProp.setFPS(60);
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      for (var cameraRecord : cameras) {
        PhotonCameraSim cameraSim = new PhotonCameraSim(cameraRecord.camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
        visionSim.addCamera(cameraSim, cameraRecord.estimator.getRobotToCameraTransform());
      }
    }
  }

  public boolean isMinAmbiguityTooHigh(EstimatedRobotPose pose) {
    double poseAmbiguity = Double.MAX_VALUE;
    for (PhotonTrackedTarget target : pose.targetsUsed) {
      if (target.poseAmbiguity != -1 && target.poseAmbiguity < poseAmbiguity) {
        poseAmbiguity = target.poseAmbiguity;
      }
    }
    // if (
    // /*poseAmbiguity*/ 0 >= kAmbiguityTolerance) {
    //   return true;
    // }

    return false;
  }

  private boolean isClosestTagTooFar(EstimatedRobotPose pose) {
    double closestTagDistance = Double.MAX_VALUE;

    for (PhotonTrackedTarget target : pose.targetsUsed) {
      double dist =
          Math.sqrt(
              Math.pow(target.bestCameraToTarget.getX(), 2)
                  + Math.pow(target.bestCameraToTarget.getY(), 2)
                  + Math.pow(target.bestCameraToTarget.getZ(), 2));
      if (dist < closestTagDistance) {
        closestTagDistance = dist;
      }
    }

    if (closestTagDistance >= kDistanceTolerance) {
      return true;
    }

    return false;
  }

  private Distance getAverageDistance(EstimatedRobotPose pose) {
    Distance averageDistance = Meters.of(0);

    for (var target : pose.targetsUsed) {
      averageDistance =
          averageDistance.plus(
              Meters.of(
                  target.bestCameraToTarget.getTranslation().getDistance(new Translation3d())));
    }

    return averageDistance.div(pose.targetsUsed.size());
  }

  private double getAverageAmbiguity(EstimatedRobotPose pose) {
    double averageAmbiguity = 0;

    for (var target : pose.targetsUsed) {
      averageAmbiguity += target.getPoseAmbiguity();
    }

    return 0.2; // averageAmbiguity / pose.targetsUsed.size();
  }

  @Override
  public void periodic() {
    for (var cameraRecord : cameras) {

      List<PhotonTrackedTarget> allTargets = new ArrayList<>();

      List<PhotonPipelineResult> data = cameraRecord.camera.getAllUnreadResults();

      for (PhotonPipelineResult result : data) {
        Optional<EstimatedRobotPose> optionalPoseResult =
            cameraRecord.estimator.estimateCoprocMultiTagPose(result);
        if (optionalPoseResult.isEmpty()) {
          optionalPoseResult = cameraRecord.estimator.estimateLowestAmbiguityPose(result);
        }
        if (optionalPoseResult.isEmpty()) {
          continue;
        }
        EstimatedRobotPose poseResult = optionalPoseResult.get();

        allTargets.addAll(poseResult.targetsUsed);

        // goes through checks to see if to discard the data
        if (!isOnField(poseResult)
            || isClosestTagTooFar(poseResult)
            || isMinAmbiguityTooHigh(poseResult)) {
          continue;
        }

        averageDistance = getAverageDistance(poseResult);

        Matrix<N3, N1> deviation =
            kBaseDeviation
                .times(Math.pow(getAverageDistance(poseResult).in(Meters), 1.5))
                .times(1 / Math.pow(poseResult.targetsUsed.size(), 2))
                .times(Math.pow(getAverageAmbiguity(poseResult) * 10, 0.75));
        swervePose =
            new VisionPose(poseResult.estimatedPose, result.getTimestampSeconds(), deviation);

        updateDrivetrain.accept(swervePose);
      }

      populateLogs(allTargets);
    }
  }

  Distance averageDistance = Units.Meters.of(0);

  public Pose3d getVisionPose() {
    return swervePose.pose;
  }

  // TODO: figure out if only using estimates with one tag makes pose beter or worse
  private boolean usingTwoTags(EstimatedRobotPose pose) {
    return (pose.targetsUsed.size() >= 2);
  }

  private boolean isOnField(EstimatedRobotPose pose) {
    return (pose.estimatedPose.getX() < kFieldWidth.in(Units.Meters))
        && (pose.estimatedPose.getX() > 0)
        && (pose.estimatedPose.getY() < kFieldHeight.in(Units.Meters))
        && (pose.estimatedPose.getY() > 0);
  }
  /*
    public Pose3d gethubLocation() {
      return kHubLocation;
    }
  */
  @Override
  public void simulationPeriodic() {
    visionSim.update(getSimPose.get());
  }

  public Transform3d[] getCameraPositions() {
    Transform3d[] tempStorage = new Transform3d[kCameraOffsets.size()];
    return kCameraOffsets.toArray(tempStorage);
  }

  // tag pose, distance, ambiguity
  private Pose3d closestTagPose = new Pose3d();
  private double closestTagDistance = -1;
  private double closestTagAmbiguity = -1;

  private Pose3d furthestTagPose = new Pose3d();
  private double furthestTagDistance = -1;
  private double furthestTagAmbiguity = -1;

  public Pose3d getClosestTagPose() {
    return closestTagPose;
  }

  public double getClosestTagDistance() {
    return closestTagDistance;
  }

  public double getClosestTagAmbiguity() {
    return closestTagAmbiguity;
  }

  public Pose3d getFurthestTagPose() {
    return furthestTagPose;
  }

  public double getFurthestTagDistance() {
    return furthestTagDistance;
  }

  public double getFurthestTagAmbiguity() {
    return furthestTagAmbiguity;
  }

  public Distance getAverageDistance() {
    return averageDistance;
  }

  public void populateLogs(List<PhotonTrackedTarget> allTargets) {
    double minDist = Double.MAX_VALUE;
    double maxDist = -1;

    for (int i = 0; i < allTargets.size(); i++) {
      PhotonTrackedTarget target = allTargets.get(i);
      double distance = target.bestCameraToTarget.getTranslation().getDistance(new Translation3d());
      Pose3d pose = kTagLayout.getTagPose(target.getFiducialId()).orElse(new Pose3d());
      double ambiguity = target.getPoseAmbiguity();

      if (distance < minDist) {
        minDist = distance;
        closestTagAmbiguity = ambiguity;
        closestTagDistance = distance;
        closestTagPose = pose;
      }
      if (distance > maxDist) {
        maxDist = distance;
        furthestTagAmbiguity = ambiguity;
        furthestTagDistance = distance;
        furthestTagPose = pose;
      }
    }
  }
}
