package frc.robot.subsystems.cameras;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NoteDetectorCamera extends SubsystemBase {
  public PhotonCamera camera;
  public Transform3d cameraOffset;
  public List<Note> notes;

  public NoteDetectorCamera(String cameraName, Transform3d cameraOffset) {
    camera = new PhotonCamera(cameraName);
    this.cameraOffset = cameraOffset;
    notes = new ArrayList<>();
    
  }

  public List<PhotonTrackedTarget> getTargets() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      return result.getTargets();
    }
    return new ArrayList<PhotonTrackedTarget>();
  }

  public Optional<PhotonTrackedTarget> getNearestTarget() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      return Optional.of(result.getBestTarget());
    }
    return Optional.empty();
  }


  public double calculateDistance(PhotonTrackedTarget target) {
    double angleToNote = Math.abs((Math.PI / 2) + cameraOffset.getRotation().getY() + Math.toRadians(target.getPitch()));
    double distance = cameraOffset.getZ() * Math.tan(angleToNote);
        
    return distance;
  }
  
  public Translation2d estimateNotePose(PhotonTrackedTarget target) {
    Pose2d robotPose = RobotContainer.swerveDrive.getPose();
    double distance = calculateDistance(target);
    System.out.println(distance);
    double noteX = robotPose.getX() + (distance * robotPose.getRotation().rotateBy(Rotation2d.fromDegrees(target.getYaw())).getCos());
    double noteY = robotPose.getY() + (distance * robotPose.getRotation().rotateBy(Rotation2d.fromDegrees(target.getYaw())).getSin());

    return new Translation2d(noteX, noteY);
  }

  public Translation2d estimateNotePoseOffcenter(PhotonTrackedTarget target) {
    Rotation2d yaw = Rotation2d.fromDegrees(target.getYaw());



    return new Translation2d();
  }

  public Pose2d rayFromTarget(PhotonTrackedTarget target) {
    return RobotContainer.swerveDrive.getPose().rotateBy(Rotation2d.fromDegrees(target.getYaw()));
  }

  @Override
  public void periodic() {
    var target = getNearestTarget();
    

    if (target.isPresent()) {
      var t = target.get();

      if (notes.isEmpty()) {
        notes.add(new Note(rayFromTarget(t), estimateNotePose(t)));
      } else {
        notes.get(0).addRay(rayFromTarget(t), estimateNotePose(t));
      }
    }
  }

  
}
