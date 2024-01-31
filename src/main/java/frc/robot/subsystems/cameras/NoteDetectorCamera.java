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

  public NoteDetectorCamera(String cameraName, Transform3d cameraOffset) {
    camera = new PhotonCamera(cameraName);
    this.cameraOffset = cameraOffset;
    notes = new ArrayList<>();
    notes.add(new Note(new Translation2d(0, 0)));

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

  public double distanceToNote(Note note) {
    Translation2d robotPose = RobotContainer.swerveDrive.getPose().getTranslation();
    return robotPose.getDistance(note.position);
  }

  public Translation2d estimateNotePose(PhotonTrackedTarget target) {
    Pose2d robotPose = RobotContainer.swerveDrive.getPose();
    double phi = -Math.toRadians(target.getYaw());
    double theta = (Math.PI / 2) - cameraOffset.getRotation().getY() - Math.toRadians(target.getPitch());

    Translation3d targetVector = new Translation3d(Math.sin(theta) * Math.cos(phi), Math.sin(theta) * Math.sin(phi),
        Math.cos(theta));
    double z = cameraOffset.getZ();
    double t = -z / targetVector.getZ();

    Translation2d noteCameraRelative = new Translation2d(targetVector.getX() * t, targetVector.getY() * t);
    Translation2d noteRobotRelative = noteCameraRelative.plus(cameraOffset.getTranslation().toTranslation2d());
    Translation2d noteFieldRelative = noteRobotRelative.rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());

    return noteFieldRelative;
  }

  public Pose2d rayFromTarget(PhotonTrackedTarget target) {
    return RobotContainer.swerveDrive.getPose().rotateBy(Rotation2d.fromDegrees(target.getYaw()));
  }

  public Note getNearestNote() {
    Note minimum = notes.get(0);

    for (Note n : notes) {
      if (distanceToNote(n) < distanceToNote(minimum)) {
        minimum = n;
      }
    }

    return minimum;
  }

  @Override
  public void periodic() {
    // if any targets are a note that already exists, update that note
    // if none are, make a new note
    // if a note is in fov and a target does not update that note's pose in this tick, remove the note

    // this is god awful if anyone has a better way to do all this please change it

    ArrayList<Note> newNotes = new ArrayList<>();
    ArrayList<Note> notesToRemove = new ArrayList<>();
    ArrayList<Note> notesInFrame = new ArrayList<>();
    ArrayList<Translation2d> detectedNotes = new ArrayList<>();
    List<Pose2d> allNotePoses = new ArrayList<>();


    // create list of notes that should be in frame
    for (Note n : notes) {
      if (n.isInFov(RobotContainer.swerveDrive.getPose())) {
        notesInFrame.add(n);
      }
    }

    // create list of positions for notes that are actually in frame
    for (PhotonTrackedTarget t : getTargets()) {
      detectedNotes.add(estimateNotePose(t));
    }

    // if any notes that should be in frame don't get updated, add them to the list of notes to be removed
    // the implication is that the robot can't see it when it should, so it's probably not there
    for (Note n : notesInFrame) {
      boolean hasBeenUpdated = false;
      // if any note gets updated, set hasBeenUpdated to true, and stop looping (can only update one note)
      for (Translation2d newPose : detectedNotes) {
        // if a detected note is new, add it to newNotes, if it is not new, it updates the note that it actually is
        if (n.addPose(newPose)) {
          hasBeenUpdated = true;
          break;
        } else {
          newNotes.add(new Note(newPose));
        }
      }

      // update remove list
      if (!hasBeenUpdated) {
        notesToRemove.add(n);
      }
    }

    // updates note list based on remove/new lists
    notes.removeAll(notesToRemove);
    notes.addAll(newNotes);

    // get notes as Pose2ds because glass takes in Pose2d list
    for (Note n : notes) {
      allNotePoses.add(new Pose2d(n.position, new Rotation2d()));
    }

    RobotContainer.field.getObject("note").setPoses(allNotePoses);
  }

}
