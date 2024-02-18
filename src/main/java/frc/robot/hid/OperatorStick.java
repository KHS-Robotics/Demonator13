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
    return this.getRawButton(8);
  }

  public boolean intakeUp() {
    return this.getRawButton(7);
  }

  public boolean intakeSetpoint() {
    return this.getRawButton(13);
  }

  public boolean shootSetpoint() {
    return this.getRawButton(14);
  }

  public boolean ampSetpoint() {
    return this.getRawButton(11);
  }

  public double climberSpeed() {
    return this.getRawAxis(1);
  }

  public boolean intake() {
    return getPOV() == 180;
  }

  public boolean outtake() {
    return getPOV() == 0;
  }
}
