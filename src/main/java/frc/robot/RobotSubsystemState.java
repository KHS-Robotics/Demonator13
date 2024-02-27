package frc.robot;

import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

public enum RobotSubsystemState {
  kIntake(IntakeState.kDown, ArmState.kIntake, ShooterState.kIntake),
  kAmp(IntakeState.kDown, ArmState.kAmp, ShooterState.kAmp),
  kShoot(IntakeState.kDown, ArmState.kShoot, ShooterState.kShoot),
  kTravel(IntakeState.kMid, ArmState.kStow, ShooterState.kShoot);

  public final IntakeState intakeState;
  public final ArmState armState;
  public final ShooterState shooterState;

  RobotSubsystemState(IntakeState intakeState, ArmState armState, ShooterState shooterState) {
    this.intakeState = intakeState;
    this.armState = armState;
    this.shooterState = shooterState;
  }
}
