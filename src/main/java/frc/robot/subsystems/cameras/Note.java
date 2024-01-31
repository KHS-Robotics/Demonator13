package frc.robot.subsystems.cameras;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class Note {
  public Translation2d position;
  private List<Translation2d> pointCloud;
  private final double MAX_ERROR_METERS = 0.5;

  public Note(Translation2d initialPose) {
    this.position = initialPose;
    this.pointCloud = new ArrayList<>();
    this.pointCloud.add(initialPose);
  }

  // attempts to add a pose to the point cloud, if it's further than the max error, reject it and return false
  // when it returns false, a new Note will be added to the note array
  public boolean addPose(Translation2d newPose) {
    // if (newPose.getDistance(position) > MAX_ERROR_METERS) {
    //   return false;
    // }

    //pointCloud.add(newPose);
    //updatePose();
    this.position = newPose;
    return true;
  }

  public boolean isInFov(Pose2d robotPose) {
    Translation2d vec = position.minus(robotPose.getTranslation());
    Rotation2d robotToNote = new Rotation2d(vec.getX(), vec.getY());
    Rotation2d yawToNote = robotPose.getRotation().rotateBy(robotToNote);

    return Math.abs(MathUtil.inputModulus(yawToNote.getDegrees(), -180, 180)) < (Constants.FRONT_NOTE_CAMERA_HFOV / 2);
  }

  private void updatePose() {
    double x = 0.0;
    double y = 0.0;

    for (Translation2d point : pointCloud) {
      x += point.getX();
      y += point.getY();
    }

    x /= pointCloud.size();
    y /= pointCloud.size();

    this.position = new Translation2d(x, y);
  }
}
