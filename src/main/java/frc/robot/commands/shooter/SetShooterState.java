// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterState;

public class SetShooterState extends Command {
  private ShooterState shooterState;
  private boolean wait;

  /** Creates a new SetShooterState. */
  public SetShooterState(ShooterState shooterState, boolean wait) {
    this.shooterState = shooterState;
    this.wait = wait;
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.shooterAngle = shooterState.angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !wait || Math.abs(Units.rotationsToDegrees(RobotContainer.shooter.getPivotAngle() - Units.rotationsToDegrees(shooterState.angle))) < 5;
  }
}
