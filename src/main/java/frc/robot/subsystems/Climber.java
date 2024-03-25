package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private CANSparkMax left, right;

  public Climber() {
    left = new CANSparkMax(RobotMap.LEFT_CLIMBER, MotorType.kBrushless);
    left.setIdleMode(IdleMode.kBrake);
    left.setSmartCurrentLimit(40);

    right = new CANSparkMax(RobotMap.RIGHT_CLIMBER, MotorType.kBrushless);
    right.setIdleMode(IdleMode.kBrake);
    right.setSmartCurrentLimit(40);
    right.setInverted(true);
  }

  public void raiseRight() {
    right.setVoltage(12);
  }

  public void raiseLeft() {
    left.setVoltage(12);
  }

  public void lowerRight() {
    right.setVoltage(-12);
  }

  public void lowerLeft() {
    left.setVoltage(-12);
  }

  public void stop() {
    left.stopMotor();
    right.stopMotor();
  }

  public void stopRight() {
    right.stopMotor();
  }

  public void stopLeft() {
    left.stopMotor();
  }
}
