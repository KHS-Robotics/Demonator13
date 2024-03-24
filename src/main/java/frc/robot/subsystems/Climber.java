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

    right = new CANSparkMax(RobotMap.RIGHT_CLIMBER, MotorType.kBrushless);
    right.setIdleMode(IdleMode.kBrake);
    right.setInverted(true);
  }

  public void raise() {
    left.setVoltage(12);
    right.setVoltage(12);
  }

  public void lower() {
    left.setVoltage(-12);
    right.setVoltage(-12);
  }

  public void stop() {
    left.stopMotor();
    right.stopMotor();
  }
}
