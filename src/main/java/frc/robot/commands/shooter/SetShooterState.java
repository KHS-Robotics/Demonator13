// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterState;

public class SetShooterState extends Command {
  private ShooterState shooterState;
  private boolean wait;

  /** Creates a new SetShooterState. */
  public SetShooterState(ShooterState shooterState, boolean wait) {
    addRequirements(RobotContainer.shooter);
    this.shooterState = shooterState;
    this.wait = wait;
  }

  public SetShooterState(ShooterState shooterState) {
    addRequirements(RobotContainer.shooter);
    this.shooterState = shooterState;
    this.wait = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.setState(shooterState);
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
    return !wait || Math.abs(RobotContainer.shooter.getPosition() - shooterState.rotations) <= 0.01;
  }
}
