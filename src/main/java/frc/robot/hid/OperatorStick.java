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
    return this.getRawButton(11);
  }

  public boolean intakeUp() {
    return this.getRawButton(5);
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
    return this.getRawButton(10);
  }

  public boolean shootSetpoint() {
    return this.getRawButton(9);
  }

  public boolean stowSetpoint() {
    return this.getRawButton(7);
  }

  public boolean ampSetpoint() {
    return this.getRawButton(8);
  }
}
