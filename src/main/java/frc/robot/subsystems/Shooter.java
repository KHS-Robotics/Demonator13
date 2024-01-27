package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooterLeader;
  private CANSparkMax shooterFollower;
  private RelativeEncoder shooterEncoder;

  private SparkPIDController shooterVelocityController;

  private CANSparkMax pivotMotor;
  private RelativeEncoder pivotEncoder;

  private PIDController pivotPositionController;

  private final double SHOOTER_GEAR_RATIO = 0;
  private final double SHOOTER_WHEEL_RADIUS = 0.0508; // 4in to meters
  private final double PIVOT_GEAR_RATIO = 0;

  public Shooter() {
    shooterLeader = new CANSparkMax(RobotMap.SHOOTER_LEADER, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(RobotMap.SHOOTER_FOLLOWER, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(RobotMap.SHOOTER_PIVOT, MotorType.kBrushless);

    shooterFollower.follow(shooterLeader);
    shooterEncoder = shooterLeader.getEncoder();
    pivotEncoder = pivotMotor.getEncoder();

    shooterVelocityController = shooterLeader.getPIDController();
    pivotPositionController = new PIDController(0, 0, 0);
  }

  public void setAngle() {

  }

  public void getAngle() {

  }

  public void setVelocity() {

  }

  public void getVelocity() {

  }

  public void calculateAngleToSpeaker() {

  }

}
