package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants {
  public static final double SENS = 0.5;

  public static final double DRIVE_VEL_ENCODER = 0.000637;
  public static final double DRIVE_POS_ENCODER = 0.000637;
  public static final double PIVOT_P = 0.01;
  public static final double PIVOT_I = 0.0;
  public static final double PIVOT_D = 0.0001;

  public static final double FRONT_LEFT_PIVOT_OFFSET_DEGREES = 225;
  public static final double FRONT_RIGHT_PIVOT_OFFSET_DEGREES = 135;
  public static final double REAR_LEFT_PIVOT_OFFSET_DEGREES = 315;
  public static final double REAR_RIGHT_PIVOT_OFFSET_DEGREES = 45;

  public static final double MAX_SPEED = 5;

  public static final double DRIVE_P = 0.01;
  public static final double DRIVE_I = 0;
  public static final double DRIVE_D = 0;
  public static final double DRIVE_KS = 0.11408;
  public static final double DRIVE_KV = 3.2717;
  public static final double DRIVE_KA = 0.17904;

  public static final double TARGET_P = 0;
  public static final double TARGET_I = 0;
  public static final double TARGET_D = 0;

  public static final Transform3d FRONT_APRILTAG_CAMERA_OFFSET = new Transform3d(0, 0, 0, new Rotation3d());
  public static final Transform3d REAR_APRILTAG_CAMERA_OFFSET = new Transform3d(0, 0, 0, new Rotation3d());

}
