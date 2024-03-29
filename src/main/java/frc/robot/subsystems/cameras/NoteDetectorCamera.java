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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NoteDetectorCamera extends SubsystemBase {
  public PhotonCamera camera;
  public Transform3d cameraOffset;
  public List<Note> notes;
  ArrayList<Pose2d> allNotePoses = new ArrayList<>();

  public NoteDetectorCamera(String cameraName, Transform3d cameraOffset) {
    camera = new PhotonCamera(cameraName);
    this.cameraOffset = cameraOffset;
    notes = new ArrayList<>();
    notes.add(new Note(new Translation2d(0, 0)));

  }

  public List<PhotonTrackedTarget> getTargets() {
    try {
      var result = camera.getLatestResult();
      if (result.hasTargets()) {
        return result.getTargets();

      }
      return new ArrayList<PhotonTrackedTarget>();
    } catch (Exception exception) {
      exception.printStackTrace();
      return new ArrayList<PhotonTrackedTarget>();
    }
  }

  public Optional<PhotonTrackedTarget> getNearestTarget() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      return Optional.of(result.getBestTarget());
    }
    return Optional.empty();
  }

  public double distanceToNote(Note note) {
    Translation2d robotPose = RobotContainer.swerveDrive.getPose().getTranslation();
    return robotPose.getDistance(note.position);
  }

  public Translation2d estimateNotePose(PhotonTrackedTarget target) {
    Pose2d robotPose = RobotContainer.swerveDrive.getPose();
    // phi is the angle on the xy plane from the x axis
    double phi = -Math.toRadians(target.getYaw() - 20);
    // theta is the angle from the z axis
    double theta = (Math.PI / 2) - cameraOffset.getRotation().getY() - Math.toRadians(target.getPitch());

    // this is a unit vector, its length is 1, but it is in the direction of the
    // note
    Translation3d targetVector = new Translation3d(Math.sin(theta) * Math.cos(phi), Math.sin(theta) * Math.sin(phi),
        Math.cos(theta));

    // z is the camera's height above the ground
    double z = cameraOffset.getZ();
    // t is the number of unit vectors needed to reach the note, like a multiplier
    double t = -z / targetVector.getZ();

    // find x and y by multiplying the x and y components of the unit vector by t
    Translation2d noteCameraRelative = new Translation2d(targetVector.getX() * t, targetVector.getY() * t);
    // rotate translation back to account for camera (now as if the camera was
    // parallel to robot's rotation)
    // Translation2d noteCameraRelativeRotated =
    // noteCameraRelative.rotateBy(Rotation2d.fromDegrees(-20));
    // remove camera offset and rotate by robot angle
    Translation2d noteRobotRelative = noteCameraRelative.plus(cameraOffset.getTranslation().toTranslation2d())
        .rotateBy(robotPose.getRotation());
    // add robot pose
    Translation2d noteFieldRelative = noteRobotRelative.plus(robotPose.getTranslation());

    return noteFieldRelative;
  }

  public Pose2d rayFromTarget(PhotonTrackedTarget target) {
    return RobotContainer.swerveDrive.getPose().rotateBy(Rotation2d.fromDegrees(target.getYaw()));
  }

  public Optional<Note> getNearestNote() {
    if (notes.isEmpty()) {
      return Optional.empty();
    }

    Note minimum = notes.get(0);

    for (Note n : notes) {
      if (distanceToNote(n) < distanceToNote(minimum)) {
        minimum = n;
      }
    }

    return Optional.of(minimum);
  }

  @Override
  public void periodic() {
    try {
      // no more caching
      notes.clear();
      allNotePoses.clear();

      for (PhotonTrackedTarget t : getTargets()) {
        notes.add(new Note(estimateNotePose(t)));
        allNotePoses.add(new Pose2d(estimateNotePose(t), new Rotation2d()));
      }

      RobotContainer.field.getObject("note").setPoses(allNotePoses);
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

}
