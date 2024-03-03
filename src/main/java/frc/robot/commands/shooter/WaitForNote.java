package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WaitForNote extends Command {
  private final Timer timer = new Timer();
  public WaitForNote() {
    addRequirements(RobotContainer.shooter, RobotContainer.intake, RobotContainer.arm);
  }

  @Override
  public void end(boolean inter) {
    timer.stop();
    timer.reset();
    RobotContainer.shooter.stopIndexer();
    RobotContainer.intake.stop();
  }

  @Override
  public boolean isFinished() {
    var hasNote = RobotContainer.shooter.hasNote();
    if (hasNote) {
      timer.start();
    } else {
      timer.reset();
    }

    return hasNote; //&& timer.hasElapsed(0.06);
  }
}
