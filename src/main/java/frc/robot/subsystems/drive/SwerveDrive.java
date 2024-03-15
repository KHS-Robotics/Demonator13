/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import java.sql.Driver;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrive extends SubsystemBase {
  public static double kMaxSpeedMetersPerSecond = 4.6;
  public static double kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
  public static double offset;
  private PIDController anglePid;
  private PIDController xPid;
  private PIDController yPid;
  public double vX;
  public double vY;
  private final Translation2d frontLeftLocation = new Translation2d(Constants.DRIVE_BASE_RADIUS_METERS,
      Constants.DRIVE_BASE_RADIUS_METERS);
  private final Translation2d frontRightLocation = new Translation2d(Constants.DRIVE_BASE_RADIUS_METERS,
      -Constants.DRIVE_BASE_RADIUS_METERS);
  private final Translation2d rearLeftLocation = new Translation2d(-Constants.DRIVE_BASE_RADIUS_METERS,
      Constants.DRIVE_BASE_RADIUS_METERS);
  private final Translation2d rearRightLocation = new Translation2d(-Constants.DRIVE_BASE_RADIUS_METERS,
      -Constants.DRIVE_BASE_RADIUS_METERS);

  public boolean fullyTrustVision = false;
  private DoubleArrayPublisher robotPosePublisher;
  private double[] poseArray = new double[] {0, 0, 0};


  public static final SwerveModule frontLeft = new SwerveModule(
      "FL",
      RobotMap.FRONT_LEFT_DRIVE,
      RobotMap.FRONT_LEFT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.FRONT_LEFT_PIVOT_ENCODER,
      Constants.FRONT_LEFT_PIVOT_OFFSET_DEGREES);
  public static final SwerveModule frontRight = new SwerveModule(
      "FR",
      RobotMap.FRONT_RIGHT_DRIVE,
      RobotMap.FRONT_RIGHT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.FRONT_RIGHT_PIVOT_ENCODER,
      Constants.FRONT_RIGHT_PIVOT_OFFSET_DEGREES);
  public static final SwerveModule rearLeft = new SwerveModule(
      "RL",
      RobotMap.REAR_LEFT_DRIVE,
      RobotMap.REAR_LEFT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.REAR_LEFT_PIVOT_ENCODER,
      Constants.REAR_LEFT_PIVOT_OFFSET_DEGREES);
  public static final SwerveModule rearRight = new SwerveModule(
      "RR",
      RobotMap.REAR_RIGHT_DRIVE,
      RobotMap.REAR_RIGHT_PIVOT,
      Constants.PIVOT_P,
      Constants.PIVOT_I,
      Constants.PIVOT_D,
      Constants.DRIVE_P,
      Constants.DRIVE_I,
      Constants.DRIVE_D,
      Constants.DRIVE_KS,
      Constants.DRIVE_KV,
      Constants.DRIVE_KA,
      RobotMap.REAR_RIGHT_PIVOT_ENCODER,
      Constants.REAR_RIGHT_PIVOT_OFFSET_DEGREES);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation,
      frontRightLocation, rearLeftLocation, rearRightLocation);

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics,
      RobotContainer.navx.getRotation2d(), new SwerveModulePosition[] {
          new SwerveModulePosition(0, new Rotation2d(frontLeft.getAngle())),
          new SwerveModulePosition(0, new Rotation2d(frontRight.getAngle())),
          new SwerveModulePosition(0, new Rotation2d(rearLeft.getAngle())),
          new SwerveModulePosition(0, new Rotation2d(rearRight.getAngle()))
      }, new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(6, 6, Double.MAX_VALUE));

  /**
   * Constructs Swerve Drive
   */
  public SwerveDrive() {
    anglePid = new PIDController(Constants.DRIVE_ANGLE_P, Constants.DRIVE_ANGLE_I, Constants.DRIVE_ANGLE_D);
    xPid = new PIDController(Constants.DRIVE_X_P, Constants.DRIVE_X_I, Constants.DRIVE_X_D);
    yPid = new PIDController(Constants.DRIVE_Y_P, Constants.DRIVE_Y_I, Constants.DRIVE_Y_D);
    anglePid.enableContinuousInput(-180.0, 180.0);
    anglePid.setTolerance(1);
    xPid.setTolerance(0.1);
    yPid.setTolerance(0.1);
    robotPosePublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic("robotPose").publish();
  }

  public void setDriveCurrentLimits(int amps) {
    frontLeft.setDriveCurrentLimit(amps);
    frontRight.setDriveCurrentLimit(amps);
    rearLeft.setDriveCurrentLimit(amps);
    rearRight.setDriveCurrentLimit(amps);
  }

  /**
   * Returns the angle of the robot as a Rotation2d as read by the navx.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees((-RobotContainer.navx.getAngle() + offset) % 360);
  }

  public void setOffset(double offset) {
    SwerveDrive.offset = offset;
  }

  /**
   * Sensitivity control for the joystick that uses a cubic function to smooth out
   * the inputs
   * instead of linear control: <code> s*x^3 + (1-s)*x </code> where s is the
   * sensitivity and x is the input.
   * https://www.wolframalpha.com/input?i=0.5%3A+s*x%5E3+%2B+%281-s%29*x+where+s+%3D+0.5
   * 
   * @param s the sensitivity from [0, 1] where 0 is full linear and 1 is full
   *          cubic
   * @return
   */
  public double sensControl(double s) {
    return Constants.JOYSTICK_SENSITIVITY * Math.pow(s, 3) + (1 - Constants.JOYSTICK_SENSITIVITY) * s;
  }

  /**
   * Method to drive the robot using joystick info. (used for teleop)
   *
   * @param xSpeed        Speed of the robot in the x direction (forward) in m/s.
   * @param ySpeed        Speed of the robot in the y direction (sideways) in m/s.
   * @param rot           Angular rate of the robot in rad/sec.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    this.vX = xSpeed;
    this.vY = ySpeed;
    if (Math.abs(rot) < 0.005 && Math.abs(xSpeed) < 0.015 && Math.abs(ySpeed) < 0.015) {
      frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(frontLeft.getAngle())));
      frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(frontRight.getAngle())));
      rearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(rearLeft.getAngle())));
      rearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(rearRight.getAngle())));
    } else {
      var swerveModuleStates = kinematics
          .toSwerveModuleStates(
              fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
                  : new ChassisSpeeds(xSpeed, ySpeed, rot));

      frontLeft.setDesiredState(swerveModuleStates[0]);
      frontRight.setDesiredState(swerveModuleStates[1]);
      rearLeft.setDesiredState(swerveModuleStates[2]);
      rearRight.setDesiredState(swerveModuleStates[3]);
    }
  }

  /**
   * used for autonomous
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0], true);
    frontRight.setDesiredState(desiredStates[1], true);
    rearLeft.setDesiredState(desiredStates[2], true);
    rearRight.setDesiredState(desiredStates[3], true);
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    vX = chassisSpeeds.vxMetersPerSecond;
    vY = chassisSpeeds.vyMetersPerSecond;
    setModuleStates(kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getAngle())));
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getAngle(), getSwerveModulePositions(), pose);
  }

  public void setPID(double p, double i, double d) {
    anglePid.setPID(p, i, d);
  }

  public void rotateToAngleInPlace(double setAngle) {
    holdAngleWhileDriving(0, 0, Rotation2d.fromDegrees(setAngle), false);
  }

  public void holdAngleWhileDriving(double x, double y, Rotation2d setAngle, boolean fieldOriented) {
    var rotateOutput = MathUtil
        .clamp(anglePid.calculate(getPose().getRotation().getDegrees(), normalizeAngle(setAngle.getDegrees())), -1, 1)
        * kMaxAngularSpeedRadiansPerSecond;
    this.drive(x, y, rotateOutput, fieldOriented);
  }

  public void holdAngleWhileDriving(double x, double y, double setAngle, boolean fieldOriented) {
    holdAngleWhileDriving(x, y, Rotation2d.fromDegrees(setAngle), fieldOriented);
  }

  public void goToPose(Pose2d target, boolean fieldOriented) {
    Pose2d pose = getPose();
    double xSpeed = MathUtil.clamp(xPid.calculate(pose.getX(), target.getX()), -1, 1) * kMaxSpeedMetersPerSecond;
    double ySpeed = MathUtil.clamp(yPid.calculate(pose.getY(), target.getY()), -1, 1) * kMaxSpeedMetersPerSecond;
    double vTheta = MathUtil
        .clamp(anglePid.calculate(normalizeAngle(pose.getRotation().getDegrees()), normalizeAngle(target.getRotation().getDegrees())), -1, 1)
        * kMaxAngularSpeedRadiansPerSecond;
    this.drive(xSpeed, ySpeed, vTheta, fieldOriented);
  }

  public boolean atSetpoint() {
    return anglePid.atSetpoint();
  }

  public boolean atSetpoint(double allowableError) {
    anglePid.setTolerance(allowableError);
    return anglePid.atSetpoint();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    // traction
    var modulePositions = getSwerveModulePositions();
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getAngle(), modulePositions);

    // vision
    if (DriverStation.isTeleopEnabled() || DriverStation.isDisabled()) {
      updateOdometryUsingFrontCamera();
      updateOdometryUsingRearCamera();
    }
  }

  private void updateOdometryUsingFrontCamera() {
    Optional<EstimatedRobotPose> estimatedFrontPose = RobotContainer.frontAprilTagCamera.getEstimatedGlobalPose();
    if (estimatedFrontPose.isPresent()) {
      var stdDevs = RobotContainer.frontAprilTagCamera
          .getEstimationStdDevs(estimatedFrontPose.get().estimatedPose.toPose2d());

      // this is stupid and we should never use it in a match!!!
      if (fullyTrustVision) {
        stdDevs.set(0, 0, 0.01);
        stdDevs.set(1, 0, 0.01);
      }

      updateOdometryUsingVisionMeasurement(estimatedFrontPose.get(), stdDevs, estimatedFrontPose.get().timestampSeconds);
    }
  }

  private void updateOdometryUsingRearCamera() {
    Optional<EstimatedRobotPose> estimatedRearPose = RobotContainer.rearAprilTagCamera.getEstimatedGlobalPose();
    if (estimatedRearPose.isPresent()) {
      var stdDevs = RobotContainer.rearAprilTagCamera
          .getEstimationStdDevs(estimatedRearPose.get().estimatedPose.toPose2d());

      // this is stupid and we should never use it in a match!!!
      if (fullyTrustVision) {
        stdDevs.set(0, 0, 0.01);
        stdDevs.set(1, 0, 0.01);
      }

      updateOdometryUsingVisionMeasurement(estimatedRearPose.get(), stdDevs, estimatedRearPose.get().timestampSeconds);
    }
  }

  // private void updateOdometryUsingFrontCamera() {
  // Optional<EstimatedRobotPose> estimatedFrontPose =
  // RobotContainer.frontAprilTagCamera.getEstimatedGlobalPose();
  // if (estimatedFrontPose.isPresent()) {
  // double stdDevXY = 0, distance = 0;
  // for (var target : estimatedFrontPose.get().targetsUsed) {
  // distance = target.getBestCameraToTarget().getTranslation().getNorm();
  // stdDevXY = distance / 2.0;
  // }

  // double distanceFromRobotPose =
  // getPose().getTranslation().getDistance(estimatedFrontPose.get().estimatedPose.getTranslation().toTranslation2d());

  // if (fullyTrustVision) {
  // stdDevXY = 0.01;
  // }
  // var stdDevs = VecBuilder.fill(stdDevXY, stdDevXY, 100000);
  // var ambiguity = 0.5;
  // if ((distance < 4 && distanceFromRobotPose < 1) || fullyTrustVision) {
  // updateOdometryUsingVisionMeasurement(estimatedFrontPose.get(), stdDevs,
  // ambiguity);
  // }
  // }
  // }

  // private void updateOdometryUsingRearCamera() {
  //   Optional<EstimatedRobotPose> estimatedRearPose = RobotContainer.rearAprilTagCamera.getEstimatedGlobalPose();
  //   if (estimatedRearPose.isPresent()) {
  //     double stdDevXY = 0, distance = 0, stdDevX = 0, stdDevY = 0;
  //     for (var target : estimatedRearPose.get().targetsUsed) {
  //       distance = target.getBestCameraToTarget().getTranslation().getNorm();
  //       // stdDevX = target.getBestCameraToTarget().getTranslation().getX() -
  //       // getPose().getX();
  //       // stdDevY = target.getBestCameraToTarget().getTranslation().getY() -
  //       // getPose().getY();
  //       stdDevXY = distance / 4.0;
  //     }

  //     double distanceFromRobotPose = getPose().getTranslation()
  //         .getDistance(estimatedRearPose.get().estimatedPose.getTranslation().toTranslation2d());

  //     if (fullyTrustVision) {
  //       stdDevXY = 0.01;
  //     }
  //     var stdDevs = VecBuilder.fill(stdDevXY, stdDevXY, 100000);
  //     var ambiguity = 1;
  //     if ((distance < 4 && distanceFromRobotPose < 1) || fullyTrustVision) {
  //       updateOdometryUsingVisionMeasurement(estimatedRearPose.get(), stdDevs, ambiguity);
  //     }
  //   }
  // }

  private void updateOdometryUsingVisionMeasurement(EstimatedRobotPose ePose, Matrix<N3, N1> visionMeasurementStdDevs,
      double timestamp) {
    poseEstimator.addVisionMeasurement(ePose.estimatedPose.toPose2d(), ePose.timestampSeconds);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return this.kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState()
    };
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
    };
  }

  public void stop() {
    frontRight.stop();
    frontLeft.stop();
    rearRight.stop();
    rearLeft.stop();
  }

  /**
   * Sets swerve modules to be all angled - useful when trying to stay still on an
   * incline.
   */
  public void lock() {
    frontRight.setDesiredState(0, 45);
    frontLeft.setDesiredState(0, -45);
    rearRight.setDesiredState(0, -45);
    rearLeft.setDesiredState(0, 45);
  }

  public void resetNavx() {
    RobotContainer.navx.reset();
  }

  public void resetPid() {
    anglePid.reset();
  }

  private static double normalizeAngle(double angle) {
    if (angle > 0) {
      angle %= 360;
      if (angle > 180) {
        angle -= 360;
      }
    } else if (angle < 0) {
      angle %= -360;
      if (angle < -180) {
        angle += 360;
      }
    }
    return angle;
  }

  public Pose2d getPoseInverted() {
    return new Pose2d(getPose().getX(), getPose().getY(), getPose().getRotation().plus(new Rotation2d(Math.PI)));
  }

  @Override
  public void periodic() {
    updateOdometry();

    var pose = getPose();
    RobotContainer.field.setRobotPose(pose);
    // poseArray[0] = pose.getX();
    // poseArray[1] = pose.getY();
    // poseArray[2] = pose.getRotation().getRadians();
    // robotPosePublisher.set(poseArray);
    

    // var yaw = RobotContainer.getRobotYaw();
    // var roll = RobotContainer.getRobotRoll();
    // var pitch = RobotContainer.getRobotPitch();
    // SmartDashboard.putNumber("Navx-Yaw", yaw);
    // SmartDashboard.putNumber("Navx-Roll", roll);
    // SmartDashboard.putNumber("Navx-Pitch", pitch);

    SmartDashboard.putNumber("Pose-X", pose.getX());
    SmartDashboard.putNumber("Pose-Y", pose.getY());
    SmartDashboard.putNumber("Pose-Radians", pose.getRotation().getRadians());
  }
}
