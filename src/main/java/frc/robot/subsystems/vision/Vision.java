package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
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
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  private List<CamAndEstimator> cameras = new ArrayList<>();

  public Consumer<VisionPose> updateDrivetrain;
  public Supplier<Pose2d> getSimPose;

  public VisionSystemSim visionSim;
  public SimCameraProperties cameraProp;

  public record CamAndEstimator(PhotonPoseEstimator estimator, PhotonCamera camera) {}

  public record VisionPose(Pose3d pose, double timestamp, Matrix<N3, N1> deviation) {}

  public Vision(Consumer<VisionPose> updateDrivetrain, Supplier<Pose2d> getSimPose) {
    this.updateDrivetrain = updateDrivetrain;
    this.getSimPose = getSimPose;

    cameras.add(
        new CamAndEstimator(
            new PhotonPoseEstimator(
                kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kCameraOffsets.get(0)),
            new PhotonCamera("aprilOne")));

    cameras.add(
        new CamAndEstimator(
            new PhotonPoseEstimator(
                kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kCameraOffsets.get(1)),
            new PhotonCamera("aprilTwo")));

    if (!RobotBase.isReal()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(kTagLayout);
      cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
      cameraProp.setCalibError(0.25, 0.08);
      cameraProp.setFPS(20);
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      for (var cameraRecord : cameras) {
        PhotonCameraSim cameraSim = new PhotonCameraSim(cameraRecord.camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
        visionSim.addCamera(cameraSim, cameraRecord.estimator.getRobotToCameraTransform());
      }
    }
  }

  public boolean isToleranceTooSmall(EstimatedRobotPose pose) {
    // outputs false if tolerance is allowed and true if tolerance is too small

    double poseAmbiguity = 1;
    double distanceAmbiguity = 1;

    for (PhotonTrackedTarget t : pose.targetsUsed) {
      if (t.poseAmbiguity != -1 && t.poseAmbiguity < poseAmbiguity) {
        poseAmbiguity = t.poseAmbiguity;
      }

      double dist =
          Math.sqrt(
              Math.pow(t.bestCameraToTarget.getX(), 2) + Math.pow(t.bestCameraToTarget.getY(), 2));

      if (dist < distanceAmbiguity) {
        distanceAmbiguity = dist;
      }
    }

    if (poseAmbiguity >= kAmbiguityTolerance) {
      return true;
    }
    if (distanceAmbiguity >= kDistanceTolerance) {
      return true;
    }

    return false;
  }

  public Distance getAverageDistance(EstimatedRobotPose pose) {
    Distance averageDistance = Meters.of(0);

    for (var target : pose.targetsUsed) {
      averageDistance.plus(
          Meters.of(target.bestCameraToTarget.getTranslation().getDistance(new Translation3d())));
    }

    return averageDistance.div(pose.targetsUsed.size());
  }

  public double getAverageAmbiguity(EstimatedRobotPose pose) {
    double averageAmbiguity = 0;

    for (var target : pose.targetsUsed) {
      averageAmbiguity += target.getPoseAmbiguity();
    }

    return averageAmbiguity / pose.targetsUsed.size();
  }

  @Override
  public void periodic() {
    update();
  }

  public boolean usingTwoTags(EstimatedRobotPose pose) {
    return (pose.targetsUsed.size() < 2);
  }

  public boolean isOnField(EstimatedRobotPose pose) {
    return (pose.estimatedPose.getX() > kFieldWidth)
        && (pose.estimatedPose.getX() < 0)
        && (pose.estimatedPose.getY() > kFieldHeight)
        && (pose.estimatedPose.getY() < 0);
  }

  public void update() {
    for (var cameraRecord : cameras) {

      List<PhotonPipelineResult> data = cameraRecord.camera.getAllUnreadResults();

      for (PhotonPipelineResult result : data) {
        Optional<EstimatedRobotPose> optionalPoseResult = cameraRecord.estimator.update(result);
        if (optionalPoseResult.isEmpty()) {
          continue;
        }
        EstimatedRobotPose poseResult = optionalPoseResult.get();

        // goes through checks to see if to discard the data
        if (usingTwoTags(poseResult) || isOnField(poseResult) || isToleranceTooSmall(poseResult)) {
          continue;
        }

        Matrix<N3, N1> deviation =
            kBaseDeviation
                .times(Math.pow(getAverageDistance(poseResult).in(Meters), 1.5))
                .times(1 / Math.pow(poseResult.targetsUsed.size(), 2))
                .times(Math.pow(getAverageAmbiguity(poseResult) * 10, 4));

        VisionPose swervePose =
            new VisionPose(poseResult.estimatedPose, result.getTimestampSeconds(), deviation);

        updateDrivetrain.accept(swervePose);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    visionSim.update(getSimPose.get());
  }
}
