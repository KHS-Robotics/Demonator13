// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeState;

public class SetIntakeState extends Command {
  private IntakeState intakeState;
  
  /** Creates a new SetIntakeState. */
  public SetIntakeState(IntakeState intakeState) {
    addRequirements(RobotContainer.intake);
    this.intakeState = intakeState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.setState(intakeState);
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
    return Math.abs(RobotContainer.intake.getPosition() - intakeState.rotations) <= 0.04;
  }
}
