package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.shooter.SetShooterState;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

/** Sets intake, arm and shooter to intake states and then performs AutoIntake. */
public class AutoPickupNote extends SequentialCommandGroup {
  // intake is already deployed or arm is clearing intake
  private Command moveIntake = new SetIntakeState(IntakeState.kDown);
  // intake is retracted and arm is blocking intake
  private Command moveArmUpThenMoveIntake = new SetArmState(ArmState.kStow).andThen(new SetIntakeState(IntakeState.kDown));
  // check if intake is down or arm is clearing intake
  private BooleanSupplier isIntakeDownOrArmUp = () -> RobotContainer.intake.isIntakeDown() || RobotContainer.arm.isArmClearingIntake();
  // Moves arm out of the way if it is blocking the intake while the intake is retracted
  private Command prepareIntakeForHandoff = new ConditionalCommand(moveIntake, moveArmUpThenMoveIntake, isIntakeDownOrArmUp);

  public AutoPickupNote() {
    addCommands(
      prepareIntakeForHandoff,
      new SetArmState(ArmState.kIntake).alongWith(new SetShooterState(ShooterState.kIntake)),
      new AutoIntake()
    );
  }
}
