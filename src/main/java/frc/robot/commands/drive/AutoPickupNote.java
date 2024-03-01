package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.shooter.SetShooterState;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

/** Sets intake, arm and shooter to intake states and then performs AutoIntake. */
public class AutoPickupNote extends SequentialCommandGroup {
  public AutoPickupNote() {
    addCommands(
      new SetIntakeState(IntakeState.kDown),
      new SetArmState(ArmState.kIntake).alongWith(new SetShooterState(ShooterState.kIntake)),
      new AutoIntake()
    );
  }
}
