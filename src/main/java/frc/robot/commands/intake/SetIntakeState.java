// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class SetIntakeState extends Command {
  private Intake intake;
  private IntakeState intakeState;
  private boolean wait;

  /** Creates a new SetIntakeState. */
  public SetIntakeState(IntakeState intakeState, boolean wait) {
    this.intake = RobotContainer.intake;
    this.intakeState = intakeState;
    this.wait = wait;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setSetpoint(intakeState);
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
    return !wait || Math.abs(Units.rotationsToDegrees(intake.getPivotAngle() - Units.rotationsToDegrees(intakeState.angle))) < 2;
  }
}
