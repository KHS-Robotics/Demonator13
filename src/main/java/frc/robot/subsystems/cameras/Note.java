package frc.robot.subsystems.cameras;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

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
