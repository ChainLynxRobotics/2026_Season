package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Vision.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.STDevCalculator;
import java.util.ArrayList;
import java.util.Arrays;
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
import org.photonvision.targeting.TargetCorner;

@Logged
public class Vision extends SubsystemBase {

  private List<CamAndEstimator> cameras = new ArrayList<>();
  @NotLogged private List<List<Double>> xSTDevData = new ArrayList<>();
  @NotLogged private List<List<Double>> ySTDevData = new ArrayList<>();
  @NotLogged private List<List<Double>> thetaSTDevData = new ArrayList<>();

  private Consumer<VisionPose> updateDrivetrain;
  private Supplier<Pose2d> getSimPose;

  private VisionSystemSim visionSim;
  private SimCameraProperties cameraProp;

  public record CamAndEstimator(PhotonPoseEstimator estimator, PhotonCamera camera) {}

  public record VisionPose(Pose3d pose, double timestamp, Matrix<N3, N1> deviation) {}

  private record TargetWithContext(
      PhotonTrackedTarget target, Pose3d visionRobotPose, Transform3d robotToCamera) {}

  @NotLogged private Supplier<Pose2d> getRobotPose;

  private int[] tagIds = new int[0];
  private double[] visionReprojErrors = new double[0];
  private double[] robotReprojErrors = new double[0];
  private int camerasProcessed = 0;
  private int framesProcessed = 0;

  @NotLogged
  private VisionPose swervePose =
      new VisionPose(new Pose3d(), 0, new Matrix<>(N3.instance, N1.instance));

  public Vision(
      Consumer<VisionPose> updateDrivetrain,
      Supplier<Pose2d> getSimPose,
      Supplier<Pose2d> getRobotPose) {
    this.updateDrivetrain = updateDrivetrain;
    this.getSimPose = getSimPose;
    this.getRobotPose = getRobotPose;

    cameras.add(
        new CamAndEstimator(
            new PhotonPoseEstimator(kTagLayout, kCameraOffsets.get(1)),
            new PhotonCamera("aprilOne")));

    cameras.add(
        new CamAndEstimator(
            new PhotonPoseEstimator(kTagLayout, kCameraOffsets.get(0)),
            new PhotonCamera("aprilTwo")));

    for (int i = 0; i < cameras.size(); i++) {
      xSTDevData.add(new ArrayList<>());
      ySTDevData.add(new ArrayList<>());
      thetaSTDevData.add(new ArrayList<>());
    }

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
    List<TargetWithContext> allTargetsWithContext = new ArrayList<>();
    camerasProcessed = 0;
    framesProcessed = 0;

    for (var cameraRecord : cameras) {
      var i = cameras.indexOf(cameraRecord);
      Transform3d robotToCamera = cameraRecord.estimator.getRobotToCameraTransform();
      boolean cameraHadResult = false;

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
        framesProcessed++;
        if (!cameraHadResult) {
          camerasProcessed++;
          cameraHadResult = true;
        }

        for (PhotonTrackedTarget target : poseResult.targetsUsed) {
          allTargetsWithContext.add(
              new TargetWithContext(target, poseResult.estimatedPose, robotToCamera));
        }

        // goes through checks to see if to discard the data
        if (!isOnField(poseResult)
            || isClosestTagTooFar(poseResult)
            || isMinAmbiguityTooHigh(poseResult)) {
          continue;
        }
        if (kKeepTrackOfSTDevs) {
          xSTDevData.get(i).add(poseResult.estimatedPose.getX());
          ySTDevData.get(i).add(poseResult.estimatedPose.getX());
          thetaSTDevData.get(i).add(poseResult.estimatedPose.getX());
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
    }

    populateLogs(allTargetsWithContext);
  }

  public void resetSTDevData() {
    xSTDevData = new ArrayList<>();
    ySTDevData = new ArrayList<>();
    thetaSTDevData = new ArrayList<>();

    for (int i = 0; i < cameras.size(); i++) {
      xSTDevData.add(new ArrayList<>());
      ySTDevData.add(new ArrayList<>());
      thetaSTDevData.add(new ArrayList<>());
    }
  }

  public double[] getXSTDDevs() {
    if (!kKeepTrackOfSTDevs) return new double[0];
    List<Double> list = new ArrayList<Double>();
    for (var data : xSTDevData) {
      list.add(STDevCalculator.calculateSTDevs(data));
    }
    return list.stream().mapToDouble(Double::doubleValue).toArray();
  }

  public double[] getYSTDDevs() {
    if (!kKeepTrackOfSTDevs) return new double[0];
    var list = new ArrayList<Double>();
    for (var data : ySTDevData) {
      list.add(STDevCalculator.calculateSTDevs(data));
    }
    return list.stream().mapToDouble(Double::doubleValue).toArray();
  }

  @Logged
  public double[] getThetaSTDDevs() {
    if (!kKeepTrackOfSTDevs) return new double[0];
    var list = new ArrayList<Double>();
    for (var data : thetaSTDevData) {
      list.add(STDevCalculator.calculateSTDevs(data));
    }
    return list.stream().mapToDouble(Double::doubleValue).toArray();
  }

  public double getSTDevSampleCount() {
    if (!kKeepTrackOfSTDevs) return 0;
    return xSTDevData.get(0).size();
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

  /**
   * Computes RMS reprojection error in pixels for a target. Projects the known 3D tag corners
   * through the given camera-to-tag transform using pinhole camera model, then compares to detected
   * 2D corners.
   */
  private double computeReprojError(PhotonTrackedTarget target, Transform3d cameraToTag) {
    List<TargetCorner> detected = target.getDetectedCorners();
    if (detected == null || detected.size() < 4) {
      return -1;
    }

    double half = kTagSizeMeters / 2.0;

    // Tag corners in tag-local 3D space (X forward/out of tag, Y left, Z up)
    // Order matches PhotonVision detected corner order:
    // 0=bottom-left, 1=bottom-right, 2=top-right, 3=top-left (in image space)
    double[][] corners3d = {
      {0, half, -half},
      {0, -half, -half},
      {0, -half, half},
      {0, half, half}
    };

    Translation3d camToTagTranslation = cameraToTag.getTranslation();
    Rotation3d camToTagRotation = cameraToTag.getRotation();

    double sumSqErr = 0;
    for (int j = 0; j < 4; j++) {
      Translation3d cornerTag =
          new Translation3d(corners3d[j][0], corners3d[j][1], corners3d[j][2]);
      Translation3d cornerCam = cornerTag.rotateBy(camToTagRotation).plus(camToTagTranslation);

      double xCam = cornerCam.getX();
      if (xCam <= 0) {
        return -1; // tag behind camera
      }

      // Pinhole projection: camera X=forward, Y=left, Z=up -> image u=right, v=down
      double u = kCameraFx * (-cornerCam.getY() / xCam) + kCameraCx;
      double v = kCameraFy * (-cornerCam.getZ() / xCam) + kCameraCy;

      double du = u - detected.get(j).x;
      double dv = v - detected.get(j).y;
      sumSqErr += du * du + dv * dv;
    }

    return Math.sqrt(sumSqErr / 4.0);
  }

  public void populateLogs(List<TargetWithContext> allTargets) {
    int maxSlots = 5;

    tagIds = new int[maxSlots];
    visionReprojErrors = new double[maxSlots];
    robotReprojErrors = new double[maxSlots];
    Arrays.fill(tagIds, -1);
    Arrays.fill(visionReprojErrors, -1);
    Arrays.fill(robotReprojErrors, -1);

    if (allTargets.isEmpty()) {
      return;
    }

    Pose3d robotPose3d = new Pose3d(getRobotPose.get());

    int detected = allTargets.size();
    int[] tempIds = new int[detected];
    double[] tempVision = new double[detected];
    double[] tempRobot = new double[detected];

    double minDist = Double.MAX_VALUE;
    double maxDist = -1;

    for (int i = 0; i < detected; i++) {
      TargetWithContext entry = allTargets.get(i);
      PhotonTrackedTarget target = entry.target;
      int id = target.getFiducialId();
      double distance = target.bestCameraToTarget.getTranslation().getDistance(new Translation3d());
      Pose3d tagPose = kTagLayout.getTagPose(id).orElse(new Pose3d());
      double ambiguity = target.getPoseAmbiguity();

      tempIds[i] = id;

      // Vision reproj error: project using the vision-estimated robot pose
      Pose3d visionCameraPose = entry.visionRobotPose.transformBy(entry.robotToCamera);
      Transform3d visionCameraToTag = new Transform3d(visionCameraPose, tagPose);
      tempVision[i] = computeReprojError(target, visionCameraToTag);

      // Robot reproj error: project using the Kalman filter pose
      Pose3d robotCameraPose = robotPose3d.transformBy(entry.robotToCamera);
      Transform3d robotCameraToTag = new Transform3d(robotCameraPose, tagPose);
      tempRobot[i] = computeReprojError(target, robotCameraToTag);

      if (distance < minDist) {
        minDist = distance;
        closestTagAmbiguity = ambiguity;
        closestTagDistance = distance;
        closestTagPose = tagPose;
      }
      if (distance > maxDist) {
        maxDist = distance;
        furthestTagAmbiguity = ambiguity;
        furthestTagDistance = distance;
        furthestTagPose = tagPose;
      }
    }

    // Sort by vision reproj error, smallest first
    Integer[] sortOrder = new Integer[detected];
    for (int i = 0; i < detected; i++) sortOrder[i] = i;
    Arrays.sort(sortOrder, (a, b) -> Double.compare(tempVision[a], tempVision[b]));

    int count = Math.min(detected, maxSlots);
    for (int i = 0; i < count; i++) {
      int idx = sortOrder[i];
      tagIds[i] = tempIds[idx];
      visionReprojErrors[i] = tempVision[idx];
      robotReprojErrors[i] = tempRobot[idx];
    }
  }

  public int getCamerasProcessed() {
    return camerasProcessed;
  }

  public int getFramesProcessed() {
    return framesProcessed;
  }

  public int[] getTagIds() {
    return tagIds;
  }

  public double[] getVisionReprojErrors() {
    return visionReprojErrors;
  }

  public double[] getRobotReprojErrors() {
    return robotReprojErrors;
  }
}
