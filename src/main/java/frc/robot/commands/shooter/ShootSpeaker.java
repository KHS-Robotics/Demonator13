package frc.robot.commands.shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  double[] optimalParams = new double[3];
  DoubleArraySubscriber optimalParamsSubscriber;
  DoubleArrayPublisher targetPosePublisher, robotPosePublisher;
  DoubleArrayTopic optimalParamsTopic, targetPoseTopic, robotPoseTopic;

  public ShootSpeaker() {
    this.addRequirements(RobotContainer.shooter, RobotContainer.swerveDrive);
    swerveDrive = RobotContainer.swerveDrive;
    shooter = RobotContainer.shooter;
    timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    if (!shooter.hasNote()) {
      hasNoteInitially = false;
      return;
    }
    hasNoteInitially = true;

    Optional<Alliance> alliance = DriverStation.getAlliance();
    hasAlliance = alliance.isPresent();
    if (!hasAlliance) {
      return;
    }

    color = alliance.get();
    if (color == Alliance.Blue) {
      targetX = 0.4572 / 2;
      targetY = 8.001 - 2.063394 - (1.05 / 2);
      targetZ = 2.05;
    } else {
      targetX = 16.5354 - (0.4572 / 2);
      targetY = 8.001 - 2.063394 - (1.05 / 2);
      targetZ = 2.05;
    }
    optimalParamsTopic = NetworkTableInstance.getDefault().getDoubleArrayTopic("optimalParams");
    targetPoseTopic = NetworkTableInstance.getDefault().getDoubleArrayTopic("targetPose");
    optimalParamsSubscriber = optimalParamsTopic.subscribe(new double[3]);
    targetPosePublisher = targetPoseTopic.publish();
    robotPosePublisher = robotPoseTopic.publish();

    targetPosePublisher.set(new double[] {targetX, targetY, targetZ});
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Pose2d robotPose = RobotContainer.swerveDrive.getPose();
    robotPosePublisher.set(new double[] {robotPose.getX(), robotPose.getY(), swerveDrive.vX, swerveDrive.vY, robotPose.getRotation().getRadians()});

    optimalParams = optimalParamsSubscriber.get();

    


    // shooter needs to go to this angle relative to the ground, not the arm
    shooter.rotationSetpoint = shooter.armRelativeToGroundRelative((optimalParams[0] / (2 * Math.PI)) + shooter.shooterFlatAngle);


    Rotation2d angleSetpoint = Rotation2d.fromRadians(optimalParams[1]).rotateBy(Rotation2d.fromDegrees(180));

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


    swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angleSetpoint, fieldRelative);
    shooter.goodTrajectory = goodTrajectory;

    if (goodTrajectory && Math.abs(robotPose.getRotation().getRadians() - optimalParams[1]) < 0.3 && Math.abs(shooter.getPosition() - optimalParams[0]) < 0.3) {
      shooter.feed();
      timer.start();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !hasNoteInitially || !hasAlliance || timer.hasElapsed(0.33);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooting();
    shooter.stopIndexer();
    swerveDrive.stop();
    timer.stop();
    timer.reset();
  }
}
