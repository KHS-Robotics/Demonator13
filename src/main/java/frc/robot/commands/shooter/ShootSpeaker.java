package frc.robot.commands.shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveDrive;

public class ShootSpeaker extends Command {
  SwerveDrive swerveDrive;
  Shooter shooter;
  Alliance color;
  boolean hasAlliance, hasNoteInitially;
  double targetX, targetY, targetZ;
  final double v0 = 20;
  boolean goodTrajectory = true;
  public Timer timer;
  Rotation2d angleSetpoint;

  public ShootSpeaker() {
    this.addRequirements(RobotContainer.swerveDrive, RobotContainer.arm, RobotContainer.shooter);
    swerveDrive = RobotContainer.swerveDrive;
    shooter = RobotContainer.shooter;
    timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    // if (!shooter.hasNote()) {
    //   hasNoteInitially = false;
    //   return;
    // }
    // hasNoteInitially = true;

    Optional<Alliance> alliance = DriverStation.getAlliance();
    hasAlliance = alliance.isPresent();
    if (!hasAlliance) {
      return;
    }

    color = alliance.get();
    if (color == Alliance.Blue) {
      targetX = 0.108472;
      targetY = 5.543668;
      targetZ = 2.05;
    } else {
      targetX = 16.452646;
      targetY = 5.543668;
      targetZ = 2.05;
    }
    shooter.setVelocity(17);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Pose2d robotPose = RobotContainer.swerveDrive.getPose();
    Translation2d vec = new Translation2d(targetX, targetY).minus(robotPose.getTranslation());
    // SmartDashboard.putNumber("targetx", targetX);
    // SmartDashboard.putNumber("targety", targetY);
    double armAngle = Shooter.shooterTable.get(vec.getNorm());
    RobotContainer.arm.setSetpoint(armAngle);
    SmartDashboard.putNumber("distance", vec.getNorm());

    angleSetpoint = Rotation2d.fromRadians(Math.atan2(vec.getY(), vec.getX())).plus(Rotation2d.fromDegrees(180));
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = 0.0;
    if (Math.abs(RobotContainer.driverController.getLeftY()) > 0.05) {
      xSpeed = swerveDrive.sensControl(-RobotContainer.driverController.getLeftY()) * SwerveDrive.kMaxSpeedMetersPerSecond;
    }

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = 0.0;
    if (Math.abs(RobotContainer.driverController.getLeftX()) > 0.05) {
      ySpeed = swerveDrive.sensControl(-RobotContainer.driverController.getLeftX()) * SwerveDrive.kMaxSpeedMetersPerSecond;
    }
    boolean fieldRelative = (RobotContainer.driverController.getRightTriggerAxis() < 0.3);


    swerveDrive.holdAngleWhileDriving(-xSpeed, -ySpeed, angleSetpoint, fieldRelative);

    // for later
    // if (RobotContainer.arm.isAtSetpoint()) {
    //   shooter.feed();
    // }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; //!hasNoteInitially || !hasAlliance || timer.hasElapsed(0.33);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    //shooter.stopShooting();
    shooter.stopIndexer();
    //swerveDrive.stop();
    timer.stop();
    timer.reset();
  }
}
