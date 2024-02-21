// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class ArmToAngle extends Command {
  private Arm arm;
  private ArmPosition setpoint;
  private double angleRadians;

  /** Creates a new ArmToAngle. */
  public ArmToAngle(ArmPosition setpoint) {
    arm = RobotContainer.arm;
    this.setpoint = setpoint;
    switch (setpoint) {
      case kAmp:  angleRadians = Math.toRadians(70);
      case kIntake: angleRadians = Math.toRadians(200);
      case kShoot:  angleRadians = Math.toRadians(180);
      case kStow:   angleRadians = Math.toRadians(120);
    }
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.goToSetpoint(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getPivotAngle() - angleRadians) < 0.3;
  }
}
