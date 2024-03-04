package frc.robot.subsystems.cameras;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagCamera extends SubsystemBase {
  public static AprilTagFieldLayout aprilTagFieldLayout;
  public PhotonPoseEstimator poseEstimator;
  public PhotonCamera camera;

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
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return poseEstimator.update();
  }
}
