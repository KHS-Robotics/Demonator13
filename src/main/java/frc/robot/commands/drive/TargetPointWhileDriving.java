package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;

public class TargetPointWhileDriving extends Command {
  private boolean fieldRelative = false;
    private Rotation2d angleSetpoint;
    private Translation2d targetPoint;

  public TargetPointWhileDriving(Translation2d target) {
    this.addRequirements(RobotContainer.swerveDrive);
    this.targetPoint = target;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    Pose2d robotPose = RobotContainer.swerveDrive.getPose();
    Translation2d vec = targetPoint.minus(robotPose.getTranslation());

    angleSetpoint = Rotation2d.fromRadians(Math.atan2(vec.getY(), vec.getX()));

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
    fieldRelative = (RobotContainer.driverController.getRightTriggerAxis() < 0.3);


    RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angleSetpoint, fieldRelative);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}
