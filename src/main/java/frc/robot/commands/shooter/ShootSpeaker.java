package frc.robot.commands.shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveDrive;

public class ShootSpeaker extends Command {
  Shooter shooter;
  Alliance color;
  boolean hasAlliance;
  double targetX, targetY, targetZ;
  final double v0 = 25;
  boolean goodTrajectory = true;
  Timer timer;

  public ShootSpeaker() {
    this.addRequirements(RobotContainer.shooter);
    shooter = RobotContainer.shooter;
    timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    hasAlliance = alliance.isPresent();
    if (hasAlliance) {
      color = alliance.get();
    }

    if (color == Alliance.Blue) {
      targetX = 0.4572 / 2;
      targetY = 8.001 - 2.063394 - (1.05 / 2);
      targetZ = 2.05;
    } else {
      targetX = 16.5354 - (0.4572 / 2);
      targetY = 8.001 - 2.063394 - (1.05 / 2);
      targetZ = 2.05;
    }

    //new LEDShoot().schedule();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    shooter.setVelocity(v0);
    Pose2d robotPose = RobotContainer.swerveDrive.getPose();

    // theta phi time
    double[] optimalParams = shooter.optimizeShooterOrientation(1,
        Math.atan2(targetY - robotPose.getY(), targetX - robotPose.getX()), 0.1, targetX, targetY, targetZ);

    // center of the exit of the shooter (middle of shooter)
    Translation3d shooterPoseRobotRelative = shooter.shooterExitRobotRelative(optimalParams[0]);
    Translation3d shooterPose = shooter.shooterExitFieldRelative(robotPose, shooterPoseRobotRelative);

    double[] in = { shooterPose.getX(), shooterPose.getY(), shooterPose.getZ(),
        RobotContainer.swerveDrive.vX + (v0 * Math.sin(Math.PI / 2 - optimalParams[0]) * Math.cos(optimalParams[1])),
        RobotContainer.swerveDrive.vY + (v0 * Math.sin(Math.PI / 2 - optimalParams[0]) * Math.sin(optimalParams[1])),
        v0 * Math.cos(Math.PI / 2 - optimalParams[0]) };

    // we may not want to actually do any of this if it can't run in real time
    double[][] trajectory = shooter.propagateWholeTrajectory3d(in, optimalParams[2], 10);
    double[] finalPoint = trajectory[trajectory.length - 1];

    if (finalPoint[5] < 0 || Math.abs(finalPoint[2] - targetZ) > 0.2) {
      goodTrajectory = false;
    } else {
      goodTrajectory = true;
    }

    // shooter needs to go to this angle relative to the ground, not the arm
    shooter.shooterAngle = shooter.armRelativeToGroundRelative((optimalParams[0] / (2 * Math.PI)) + shooter.shooterFlatAngle);


    Rotation2d angleSetpoint = Rotation2d.fromRadians(optimalParams[1]).rotateBy(Rotation2d.fromDegrees(180));

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = 0.0;
    if (Math.abs(RobotContainer.driverController.getLeftY()) > 0.05) {
      xSpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftY()) * SwerveDrive.kMaxSpeedMetersPerSecond;
    }

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = 0.0;
    if (Math.abs(RobotContainer.driverController.getLeftX()) > 0.05) {
      ySpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftX()) * SwerveDrive.kMaxSpeedMetersPerSecond;
    }
    boolean fieldRelative = (RobotContainer.driverController.getRightTriggerAxis() < 0.3);


    RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angleSetpoint, fieldRelative);

    if (goodTrajectory && Math.abs(robotPose.getRotation().getRadians() - optimalParams[1]) < 0.3 && Math.abs(shooter.getPivotAngle() - optimalParams[0]) < 0.3) {
      shooter.feed();
      timer.start();
    }


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (!hasAlliance || timer.hasElapsed(1) || !shooter.hasNote()) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    shooter.setVelocity(0);
    shooter.stopIndexer();
  }
}
