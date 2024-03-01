/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RotateToAngle extends Command {
  private static final double kErrorInDegrees = 3;
  private Timer timer = new Timer();

  private final DoubleSupplier angle;

  public RotateToAngle(DoubleSupplier angle) {
    addRequirements(RobotContainer.swerveDrive);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.resetPid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.rotateToAngleInPlace(angle.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var atSetpoint = RobotContainer.swerveDrive.atSetpoint(kErrorInDegrees);
    if (atSetpoint) {
      timer.start();
    } else {
      timer.reset();
    }

    return atSetpoint && timer.hasElapsed(0.20);
  }
}