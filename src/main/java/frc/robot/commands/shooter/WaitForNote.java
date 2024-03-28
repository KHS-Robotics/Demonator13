package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WaitForNote extends Command {
  public WaitForNote() {
    addRequirements(RobotContainer.shooter, RobotContainer.intake);
  }

  @Override
  public void end(boolean inter) {
    RobotContainer.shooter.stopIndexer();
    RobotContainer.intake.stop();
  }

  @Override
  public boolean isFinished() {
    var hasNote = RobotContainer.shooter.hasNote();
    return hasNote;
  }
}
