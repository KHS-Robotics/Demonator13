package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants {
  public static final double SENS = 0.5;

  public static final double WHEEL_RADIUS_METERS = 0.0508;
  public static final double SDS_L2_DRIVE_GEARING = 6.75;
  public static final double DRIVE_POS_ENCODER = (2 * Math.PI * WHEEL_RADIUS_METERS) / SDS_L2_DRIVE_GEARING;
  public static final double DRIVE_VEL_ENCODER = DRIVE_POS_ENCODER / 60.0;

  public static final double PIVOT_P = 0.01;
  public static final double PIVOT_I = 0.0;
  public static final double PIVOT_D = 0.0001;

  public static final double FRONT_LEFT_PIVOT_OFFSET_DEGREES = 225;
  public static final double FRONT_RIGHT_PIVOT_OFFSET_DEGREES = 135;
  public static final double REAR_LEFT_PIVOT_OFFSET_DEGREES = 315;
  public static final double REAR_RIGHT_PIVOT_OFFSET_DEGREES = 45;

  public static final double DRIVE_P = 0.01;
  public static final double DRIVE_I = 0;
  public static final double DRIVE_D = 0;
  public static final double DRIVE_KS = 0.11408;
  public static final double DRIVE_KV = 3.2717;
  public static final double DRIVE_KA = 0.17904;

  public static final double DRIVE_ANGLE_P = 0.009;
  public static final double DRIVE_ANGLE_I = 0;
  public static final double DRIVE_ANGLE_D = 0;

  public static final double DRIVE_X_P = 0.3;
  public static final double DRIVE_X_I = 0;
  public static final double DRIVE_X_D = 0;

  public static final double DRIVE_Y_P = 0.3;
  public static final double DRIVE_Y_I = 0;
  public static final double DRIVE_Y_D = 0;

  public static final Transform3d FRONT_APRILTAG_CAMERA_OFFSET = new Transform3d(0, 0, 0.7112, new Rotation3d(0, Math.toRadians(-20), 0));
  public static final Transform3d REAR_APRILTAG_CAMERA_OFFSET = new Transform3d(0, 0, 0, new Rotation3d());
  public static final double FRONT_NOTE_CAMERA_HFOV = 53;

  public static final double INTAKE_RADIUS = 0.5;

  public static final int LED_LENGTH = 40;

  public static final double INTAKE_PIVOT_P = 0;
  public static final double INTAKE_PIVOT_I = 0;
  public static final double INTAKE_PIVOT_D = 0;

  public static final double INTAKE_TOP = 0.582;
  public static final double INTAKE_BOTTOM = 0.223;
}
