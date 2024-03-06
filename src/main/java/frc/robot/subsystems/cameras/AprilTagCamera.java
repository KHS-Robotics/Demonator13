package frc.robot.subsystems.cameras;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagCamera extends SubsystemBase {
  public static AprilTagFieldLayout aprilTagFieldLayout;
  public PhotonPoseEstimator poseEstimator;
  public PhotonCamera camera;

  private double lastEstTimestamp = 0;

  static {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException exception) {
      exception.printStackTrace();
      DriverStation.reportError("Failed to load april tag field", false);
    }
  }

  public AprilTagCamera(String cameraName, Transform3d cameraOffset) {
    camera = new PhotonCamera(cameraName);
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
        cameraOffset);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = poseEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult)
      lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from
   * {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    if (!getLatestResult().hasTargets()) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    var estStdDevs = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0)
      return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

}
