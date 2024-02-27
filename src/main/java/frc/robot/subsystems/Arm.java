package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
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
  private ArmFeedforward armFf;

  private double kP = 40;
  private double kI = 2;
  private double kD = 1;

  private double kS = 0.26602;
  private double kG = 0.7;
  private double kV = 4.5387;
  private double kA = 0.27838;

  public double armPosition = 0;

  public Arm() {
    pivotMotor = new CANSparkMax(RobotMap.ARM_PIVOT, MotorType.kBrushless);
    pivotMotor.setInverted(false);
    pivotFollower = new CANSparkMax(RobotMap.ARM_FOLLOWER, MotorType.kBrushless);
    pivotFollower.follow(pivotMotor, true);
    
    
    pivotEncoder = new CANcoder(RobotMap.ARM_CANCODER);
    
    //pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    //pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    
    armPid = new PIDController(kP, kI, kD);
    armFf = new ArmFeedforward(kS, kG, kV, kA);
  }

  public void goToAngle(double angle) {
    double pidOutput = armPid.calculate(getPivotAngle(), angle);
    double ffOutput = armFf.calculate(angle, 0);
    
    pivotMotor.setVoltage(pidOutput + ffOutput);
    System.out.println(pidOutput + ffOutput);
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

  public enum ArmState {
    kStow(0.618),
    kIntake(0.8),
    kShoot(0.75),
    kAmp(0.5);

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
    SmartDashboard.putNumber("armAngle", Units.rotationsToDegrees(getPivotAngle()));
    SmartDashboard.putNumber("armSetpoint", Units.rotationsToDegrees(armPosition));
    SmartDashboard.putNumber("armError", Units.rotationsToDegrees((Math.abs(armPosition - getPivotAngle()))));
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