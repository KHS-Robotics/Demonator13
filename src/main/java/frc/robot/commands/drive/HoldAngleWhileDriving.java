package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;

public class HoldAngleWhileDriving extends Command {
    private boolean fieldRelative = false;
    private Rotation2d angleSetpoint;

  public HoldAngleWhileDriving(boolean fod) {
    this.addRequirements(RobotContainer.swerveDrive);
    this.fieldRelative = fod;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    angleSetpoint = RobotContainer.swerveDrive.getPose().getRotation();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
    fieldRelative = RobotContainer.driverController.getHID().getRightTriggerAxis() > 0.5;

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = 0.0;
    if (Math.abs(RobotContainer.driverController.getLeftY()) > 0.05) {
      xSpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftY())
          * SwerveDrive.kMaxSpeedMetersPerSecond;
    }

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = 0.0;
    if (Math.abs(RobotContainer.driverController.getLeftX()) > 0.05) {
      ySpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getLeftX())
          * SwerveDrive.kMaxSpeedMetersPerSecond;
    }

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = 0;
    if (Math.abs(RobotContainer.driverController.getRightX()) > 0.05) {
      rot = RobotContainer.swerveDrive.sensControl(-RobotContainer.driverController.getRightX())
          * SwerveDrive.kMaxAngularSpeedRadiansPerSecond;
    }

    var sign = fieldRelative ? 1 : -1;
    RobotContainer.swerveDrive.holdAngleWhileDriving(-xSpeed*sign, -ySpeed*sign, angleSetpoint, fieldRelative);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stop();
  }
}
