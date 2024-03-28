package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private CANSparkMax pivotFollower;
  private CANcoder pivotEncoder;
  private PIDController armPid;

  private double kP = 32;
  private double kI = 8;
  private double kD = 0;

  private double kG = 0.7;

  public double rotationSetpoint = ArmState.kStow.rotations;

  public Arm() {
    pivotMotor = new CANSparkMax(RobotMap.ARM_PIVOT, MotorType.kBrushless);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(false);
    
    pivotFollower = new CANSparkMax(RobotMap.ARM_FOLLOWER, MotorType.kBrushless);
    pivotFollower.setIdleMode(IdleMode.kBrake);
    pivotFollower.follow(pivotMotor, true);

    pivotMotor.setSmartCurrentLimit(45);
    pivotFollower.setSmartCurrentLimit(45);
    
    pivotEncoder = new CANcoder(RobotMap.ARM_CANCODER);

    
    
    // pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    
    armPid = new PIDController(kP, kI, kD);
    armPid.setIZone(0.1);
    armPid.setTolerance(0.01);
  }

  public void goToSetpoint(double angle) {
    double pidOutput = armPid.calculate(getPosition(), angle);
    double gravityOutput = kG * Math.cos(Units.rotationsToRadians(angle - 0.25));
    
    var output = pidOutput + gravityOutput;
    pivotMotor.setVoltage(output);
  }

  public void setVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public void setState(ArmState state) {
    setSetpoint(state.rotations);
  }

  public void setSetpoint(double setpoint) {
    rotationSetpoint = setpoint;
    this.armPid.reset();
  }

  public double getPosition() {
    return pivotEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean isAtState(ArmState state) {
    return Math.abs(getPosition() - state.rotations) <= 0.01;
  }

  public boolean isAtSetpoint() {
    return armPid.atSetpoint();
  }

  public boolean isArmClearingIntake() {
    return getPosition() < 0.69;
  }

  public enum ArmState {
    kStow(0.63),
    kDeployDemonHorns(0.605),
    kIntake(0.827),
    kShoot(0.75),
    kShootFromSubwoofer(0.75),
    kShootFromSubwooferAuto(0.7575),
    kShootFromPodium(0.7075),
    kAmp(0.513),
    kFeedFromCenter(0.75);

    public final double rotations;

    ArmState(double rotations) {
      this.rotations = rotations;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("armAngle", getPosition());
    SmartDashboard.putNumber("armSetpoint", rotationSetpoint);
    SmartDashboard.putNumber("armError", Math.abs(rotationSetpoint - getPosition()));
    // SmartDashboard.putBoolean("armAtSetpoint", Math.abs(rotationSetpoint - getPosition()) <= 0.02);

    // kG = SmartDashboard.getNumber("kG", kG);
    // kP = SmartDashboard.getNumber("kP", kP);
    // kI = SmartDashboard.getNumber("kI", kI);
    // kD = SmartDashboard.getNumber("kD", kD);
    // SmartDashboard.putNumber("kG", kG);
    // SmartDashboard.putNumber("kP", kP);
    // SmartDashboard.putNumber("kI", kI);
    // SmartDashboard.putNumber("kD", kD);

    // armPid.setPID(kP, kI, kD);

    goToSetpoint(rotationSetpoint);
  }
}