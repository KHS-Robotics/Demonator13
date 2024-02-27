package frc.robot.hid;

import edu.wpi.first.wpilibj.Joystick;

public class OperatorStick extends Joystick {
  public OperatorStick(int port) {
    super(port);
  }

  public boolean shoot() {
    return this.getRawButton(1);
  }

  public boolean intakeDown() {
    return this.getRawButton(10);
  }

  public boolean intakeUp() {
    return this.getRawButton(5);
  }

  public boolean armIntakeSetpoint() {
    return this.getRawButton(9);
  }

  public boolean armShootSetpoint() {
    return this.getRawButton(7);
  }

  public boolean armAmpSetpoint() {
    return this.getRawButton(6);
  }

  public boolean armStowSetpoint() {
    return this.getRawButton(8);
  }

  public boolean wristIntake() {
    return this.getRawButton(13);
  }

  public boolean wristShoot() {
    return this.getRawButton(14);
  }

  public double climberSpeed() {
    return this.getRawAxis(1);
  }

  public boolean fastUp() {
    return this.getRawButton(16);
  }

  public boolean index() {
    return getPOV() == 270;
  }

  public boolean outdex() {
    return getPOV() == 90;
  }

  public boolean intake() {
    return getPOV() == 180;
  }

  public boolean outtake() {
    return getPOV() == 0;
  }

  public boolean intakeSetpoint() {
    return false;
  }

  public boolean shootSetpoint() {
    return false;
  }

  public boolean stowSetpoint() {
    return false;
  }

  public boolean ampSetpoint() {
    return false;
  }
}
