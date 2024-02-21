// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterAngle;
import frc.robot.subsystems.Shooter;

public class AngleShooter extends Command {
  private Shooter shooter;
  private ShooterAngle setpoint;
  private double angleRadians;

  /** Creates a new ArmToAngle. */
  public AngleShooter(ShooterAngle setpoint) {
    shooter = RobotContainer.shooter;
    this.setpoint = setpoint;
    switch (setpoint) {
      case kAmp:  angleRadians = Math.toRadians(90);
      case kIntake: angleRadians = Math.toRadians(60);
      case kShoot:  angleRadians = Math.toRadians(45);
    }
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.goToSetpoint(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(shooter.getPivotAngle() - angleRadians) < 0.3;
  }
}
