package frc.robot.commands.shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShootSpeaker extends Command {
  Shooter shooter;
  Alliance color;
  boolean hasAlliance;
  double targetX, targetY, targetZ;
  final double v0 = 15;
  boolean goodTrajectory = false;

  public ShootSpeaker() {
    this.addRequirements(RobotContainer.shooter);
    shooter = RobotContainer.shooter;
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
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Pose2d robotPose = RobotContainer.swerveDrive.getPose();

    double[] optimalParams = shooter.optimizeShooterOrientation(1,
        Math.atan2(targetY - robotPose.getY(), targetX - robotPose.getX()), 0.1, targetX, targetY, targetZ);

    Translation3d shooterPoseRobotRelative = shooter.shooterExitRobotRelative(optimalParams[0]);
    Translation3d shooterPose = shooter.shooterExitFieldRelative(robotPose, shooterPoseRobotRelative);

    double[] in = { shooterPose.getX(), shooterPose.getY(), shooterPose.getZ(),
        RobotContainer.swerveDrive.vX + (v0 * Math.sin(Math.PI / 2 - optimalParams[0]) * Math.cos(optimalParams[1])),
        RobotContainer.swerveDrive.vY + (v0 * Math.sin(Math.PI / 2 - optimalParams[0]) * Math.sin(optimalParams[1])),
        v0 * Math.cos(Math.PI / 2 - optimalParams[0]) };

    double[][] trajectory = shooter.propagateWholeTrajectory3d(in, optimalParams[2], 10);
    double[] finalPoint = trajectory[trajectory.length - 1];

    if (finalPoint[5] < 0 || Math.abs(finalPoint[2] - targetZ) > 0.2) {
      goodTrajectory = false;
    } else {
      goodTrajectory = true;
    }

    


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (!hasAlliance) {
      return false;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
