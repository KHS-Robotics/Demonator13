package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;

public class HoldAngleWhileDriving extends Command {
    private Rotation2d angleSetpoint;

  public HoldAngleWhileDriving() {
    this.addRequirements(RobotContainer.swerveDrive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    angleSetpoint = RobotContainer.swerveDrive.getPose().getRotation();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
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

    var fieldRelative = RobotContainer.driverController.getHID().getRightTriggerAxis() > 0.5;
    var sign = fieldRelative && !DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1;
    RobotContainer.swerveDrive.holdAngleWhileDriving(sign*xSpeed, sign*ySpeed, angleSetpoint, fieldRelative);
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
