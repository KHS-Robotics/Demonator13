package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.cameras.Note;

public class AutoIntake extends Command {
  private boolean fieldRelative = false;
  private Optional<Note> target;
  private Pose2d robotTarget;
  private Pose2d robotPose;

  public AutoIntake() {
    this.addRequirements(RobotContainer.swerveDrive);
    //addRequirements intake, arm?
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    this.robotPose = RobotContainer.swerveDrive.getPose();

    // update target based on camera
    this.target = RobotContainer.frontNoteCamera.getNearestNote();

    if (this.target.isEmpty()) {
      return;
    }

    // vector from robot to target (imagine the robot center is at 0,0)
    Translation2d vec = target.get().position.minus(robotPose.getTranslation());

    // angle from robot center to note
    Rotation2d angleToNote = new Rotation2d(vec.getX(), vec.getY());

    // vector from robot center to note, but shifted so that the intake will be on the center of the note
    // this is our actual x,y target that we need to drive to, angleToNote is our actual angle that we need to face
    vec = vec.minus(new Translation2d(angleToNote.getCos() * Constants.INTAKE_RADIUS, angleToNote.getSin() * Constants.INTAKE_RADIUS));

    // this is the field position of our x,y target, and rotation, our final target
    this.robotTarget = new Pose2d(vec.plus(robotPose.getTranslation()), angleToNote);

    // set arm/intake to intake setpoint
    // continuously run intake wheels

    // drive to robotTarget
    fieldRelative = (RobotContainer.driverController.getRightTriggerAxis() < 0.3);
    RobotContainer.swerveDrive.goToPose(robotTarget, fieldRelative);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // check for note in intake tripped beambreak?
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    // stow
    RobotContainer.swerveDrive.stop();
  }
}
