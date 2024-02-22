package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeSetpoint;
import frc.robot.subsystems.drive.SwerveDrive;

public class AngleIntake extends Command {
  private Intake intake;
  private double rotations;

  /** Creates a new ArmToAngle. */
  public AngleIntake(IntakeSetpoint setpoint) {
    this.intake = RobotContainer.intake;
    rotations = setpoint.angle;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.goToAngle(rotations);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     System.out.println(intake.getPivotAngle());
    return Math.abs(intake.getPivotAngle() - rotations) < 0.01;
  }
}

