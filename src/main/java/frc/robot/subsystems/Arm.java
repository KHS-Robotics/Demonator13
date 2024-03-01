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

  private double kP = 40;
  private double kI = 3;
  private double kD = 1;

  private double kG = 0.4;

  public double armPosition = 0;

  public Arm() {
    pivotMotor = new CANSparkMax(RobotMap.ARM_PIVOT, MotorType.kBrushless);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(false);
    
    pivotFollower = new CANSparkMax(RobotMap.ARM_FOLLOWER, MotorType.kBrushless);
    pivotFollower.setIdleMode(IdleMode.kBrake);
    pivotFollower.follow(pivotMotor, true);
    
    pivotEncoder = new CANcoder(RobotMap.ARM_CANCODER);
    
    //pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    //pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    
    armPid = new PIDController(kP, kI, kD);
  }

  public void goToAngle(double angle) {
    double pidOutput = armPid.calculate(getPivotAngle(), angle);
    double gravityOutput = kG * Math.sin(Units.rotationsToRadians(angle));
    
    var output = pidOutput + gravityOutput;
    pivotMotor.setVoltage(output);
  }

  public void setVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public void goToSetpoint(ArmState setpoint) {
    setSetpoint(setpoint);
  }

  public double getPivotAngle() {
    return pivotEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean isArmClearingIntake() {
    return armPosition < 0.75;
  }

  public enum ArmState {
    kStow(0.618),
    kIntake(0.83),
    kShoot(0.75),
    kAmp(0.52);

    public final double angle;

    ArmState(double rotations) {
      this.angle = rotations;
    }
  }

  public void resetPID() {
    this.armPid.reset();
  }

  public void setSetpoint(ArmState setpoint) {
    armPosition = setpoint.angle;
    resetPID();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("armAngle", getPivotAngle());
    SmartDashboard.putNumber("armSetpoint", armPosition);
    SmartDashboard.putNumber("armError", Math.abs(armPosition - getPivotAngle()));
    //kG = SmartDashboard.getNumber("kG", kG);
    // kP = SmartDashboard.getNumber("kP", kP);
    // kI = SmartDashboard.getNumber("kI", kI);
    // kD = SmartDashboard.getNumber("kD", kD);
    // SmartDashboard.putNumber("kG", kG);
    // SmartDashboard.putNumber("kP", kP);
    // SmartDashboard.putNumber("kI", kI);
    // SmartDashboard.putNumber("kD", kD);

    //armPid.setPID(kP, kI, kD);
    


    goToAngle(armPosition);
  }
}