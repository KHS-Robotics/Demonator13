package frc.robot.subsystems.cameras;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Note {
    Translation2d estimatedPose;
    ArrayList<Pose2d> recentRays;
    ArrayList<Translation2d> pointCloud;
    private int MAX_RAYS = 5;

    public Note(Pose2d initialRay, Translation2d initialPoseGuess) {
        estimatedPose = initialPoseGuess;
        pointCloud = new ArrayList<>();
        recentRays = new ArrayList<>();
        addRay(initialRay, initialPoseGuess);
    }

    public void addRay(Pose2d ray, Translation2d poseGuess) {
        // if distance is off by >0.1m disregard this ray
        if (Math.abs(ray.getTranslation().getDistance(poseGuess) - ray.getTranslation().getDistance(estimatedPose)) > 0.1) {
            return;
        }

        for (Pose2d otherRay : recentRays) {
            // if parallel to any other ray, disregard
            if (ray.getRotation().equals(otherRay.getRotation())) {
                return;
            }
        }

        if (recentRays.size() == MAX_RAYS) {
            recentRays.remove(0);
        }

        recentRays.add(ray);

        updatePointCloud();
        estimatedPose = averageOfPointCloud();
    }

    private void updatePointCloud() {
        pointCloud.clear();
        for (Pose2d ray : recentRays) {
            for (Pose2d rayToIntersect : recentRays) {
                if (!ray.equals(rayToIntersect)) {
                    pointCloud.add(intersect(ray, rayToIntersect));
                }
            }
        }
    }

    private Translation2d averageOfPointCloud() {
        double x = 0.0;
        double y = 0.0;

        for (Translation2d point : pointCloud) {
            x += point.getX();
            y += point.getY();
        }

        x /= pointCloud.size();
        y /= pointCloud.size();

        return new Translation2d(x, y);
    }

    private Translation2d intersect(Pose2d a, Pose2d b) {
        double ma = a.getRotation().getTan();
        double mb = b.getRotation().getTan();

        double ax = a.getX();
        double ay = a.getY();
        double bx = b.getX();
        double by = b.getY();

        double x = ((ma * ax) - (mb * bx) + by - ay) / (ma - mb);
        double y = ma * (x - ax) + ay;

        return new Translation2d(x, y);
    }
}
