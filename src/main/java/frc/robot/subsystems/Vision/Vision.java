package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Vision.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public Vision(Consumer<VisionPose> updateDrivetrain, Supplier<Pose2d> getSimPose) {
    this.updateDrivetrain = updateDrivetrain;
    this.getSimPose = getSimPose;

    cameras.add(
        new CamAndEstimator(
            new PhotonPoseEstimator(kTagLayout, kCameraOffsets.get(0)),
            new PhotonCamera("aprilOne")));

    // cameras.add(
    //     new CamAndEstimator(
    //         new PhotonPoseEstimator(
    //             kTagLayout, kCameraOffsets.get(1)),
    //         new PhotonCamera("aprilTwo")));

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
    double poseAmbiguity = 1;
    for (PhotonTrackedTarget target : pose.targetsUsed) {
      if (target.poseAmbiguity != -1 && target.poseAmbiguity < poseAmbiguity) {
        poseAmbiguity = target.poseAmbiguity;
      }
    }
    if (poseAmbiguity >= kAmbiguityTolerance) {
      System.out.println("Ambiguity to high");
      return true;
    }

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
        System.out.println("Closest tag to far");
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

    return averageAmbiguity / pose.targetsUsed.size();
  }

  @Override
  public void periodic() {
    for (var cameraRecord : cameras) {

      List<PhotonPipelineResult> data = cameraRecord.camera.getAllUnreadResults();

      for (PhotonPipelineResult result : data) {
        Optional<EstimatedRobotPose> optionalPoseResult =
            cameraRecord.estimator.estimateCoprocMultiTagPose(result);
        if (optionalPoseResult.isEmpty()) {
          System.out.println("No results");
          continue;
        }
        EstimatedRobotPose poseResult = optionalPoseResult.get();

        if (!isOnField(poseResult)) {
          System.out.println("Not on field");
        }

        // goes through checks to see if to discard the data
        if (!isOnField(poseResult)
            || isClosestTagTooFar(poseResult)
            || isMinAmbiguityTooHigh(poseResult)) {
          continue;
        }

        Matrix<N3, N1> deviation =
            kBaseDeviation
                .times(Math.pow(getAverageDistance(poseResult).in(Meters), 1.5))
                .times(1 / Math.pow(poseResult.targetsUsed.size(), 2))
                .times(Math.pow(getAverageAmbiguity(poseResult) * 10, 0.75));
        System.out.println(deviation);
        VisionPose swervePose =
            new VisionPose(poseResult.estimatedPose, result.getTimestampSeconds(), deviation);

        updateDrivetrain.accept(swervePose);
      }
    }
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
}
