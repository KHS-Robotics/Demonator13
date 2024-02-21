package frc.robot.commands.intake;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeSetpoint;

public class AngleIntake extends Command {
  private Intake intake;
  private IntakeSetpoint setpoint;
  private double angleRadians;

  /** Creates a new ArmToAngle. */
  public AngleIntake(IntakeSetpoint setpoint) {
    this.intake = RobotContainer.intake;
    this.setpoint = setpoint;
    switch (setpoint) {
      case kUp:  angleRadians = Math.toRadians(0);
      case kDown: angleRadians = Math.toRadians(60);
    }
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.goToSetpoint(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(intake.getPivotAngle() - angleRadians) < 0.3;
  }
}

