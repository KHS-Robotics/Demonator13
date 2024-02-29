package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Ramps the shooter up then indexes the note into the shooter. */
public class RampShooterThenIndex extends SequentialCommandGroup {
  public RampShooterThenIndex(DoubleSupplier velocity) {
    addCommands(
      new RampShooter(velocity),
      new InstantCommand(() -> RobotContainer.shooter.index())
    );
  }
}
