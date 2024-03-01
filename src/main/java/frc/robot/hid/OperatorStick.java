package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;

/** Thrustmaster T.16000M FCS */
public class OperatorStick extends Joystick {
  public OperatorStick(int port) {
    super(port);
  }

  public boolean outtakeNote() {
    return getPOV() == 0;
  }

  public boolean intakeNote() {
    return getPOV() == 180;
  }

  public boolean scoreSpeaker() {
    return this.getRawButton(1);
  }

  public boolean scoreAmp() {
    return this.getRawButton(2);
  }
  
  public boolean deployIntake() {
    return this.getRawButton(5);
  }

  public boolean stowSetpoint() {
    return this.getRawButton(7);
  }

  public boolean ampSetpoint() {
    return this.getRawButton(8);
  }

  public boolean shootSetpoint() {
    return this.getRawButton(9);
  }

  public boolean intakeNoteSetpoint() {
    return this.getRawButton(10);
  }

  public boolean retractIntake() {
    return this.getRawButton(11);
  }
}
