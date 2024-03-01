package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RampShooter extends Command {
  private final Timer timer = new Timer();
  private final double kTimeAtSetpointForCompletionInSeconds = 0.15;
  private final double kVelocityTolerance = 1.0;

  private final DoubleSupplier velocity;

  public RampShooter(DoubleSupplier velocity) {
    this.addRequirements(RobotContainer.shooter);
    this.velocity = velocity;
  }

  @Override
  public void initialize() {
    RobotContainer.shooter.setVelocity(velocity.getAsDouble());
    timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return this.isAtSetpointForSpecifiedPeriodOfTime(kTimeAtSetpointForCompletionInSeconds);
  }

  /** Ensures shooter is truly at the setpoint and not just for a blip in time. */
  private boolean isAtSetpointForSpecifiedPeriodOfTime(double time) {
    var atSetpoint = RobotContainer.shooter.isShooterRampedUp(kVelocityTolerance);
    if (atSetpoint) {
      timer.start();
    } else {
      timer.reset();
    }

    return atSetpoint && timer.hasElapsed(time);
  }
}
