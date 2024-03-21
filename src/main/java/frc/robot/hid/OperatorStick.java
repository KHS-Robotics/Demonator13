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

  public boolean shootManual() {
    return this.getRawButton(1);
  }

  public boolean scoreAmp() {
    return this.getRawButton(2);
  }
  
  public boolean deployIntake() {
    return this.getRawButton(16);
  }

  public boolean retractIntake() {
    return this.getRawButton(14);
  }

  public boolean midIntake() {
    return this.getRawButton(15);
  }

  public boolean ampArm() {
    return this.getRawButton(5);
  }

  public boolean stowArm() {
    return this.getRawButton(6);
  }

  public boolean handoffArm() {
    return this.getRawButton(7);
  }

  public boolean subwooferArm() {
    return this.getRawButton(10);
  }

  public boolean podiumArm() {
    return this.getRawButton(9);
  }

  public boolean levelArm() {
    return this.getRawButton(8);
  }

  public boolean fullyTrustVision() {
    return this.getRawButton(11);
  }

  public boolean getShooterRamping() {
    return this.getRawAxis(3) < -0.99;
  }

  public boolean feedShotSetpoint() {
    return this.getRawButton(12);
  }

  public boolean forceIndex() {
    return this.getRawButton(11);
  }
/* 
  public boolean stageArm() {
    return this.getRawButton();
  }

  public boolean stageIntake() {
    return this.getRawButton();
  }
  */
}
