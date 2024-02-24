package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private CANSparkMax pivotFollower;
  private CANcoder pivotEncoder;
  private final double gearRatio = 0.0;
  private PIDController armPid;
  private ArmFeedforward armFf;
  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.5, 5);

  private double kP = 40;
  private double kI = 2;
  private double kD = 1;

  private double kS = 0.026602;
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
  }

  public void goToSetpoint(ArmPosition setpoint) {
    goToAngle(setpoint.angle);
  }

  public double getPivotAngle() {
    return pivotEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public enum ArmPosition {
    kStow(Units.degreesToRotations(120)),
    kIntake(Units.degreesToRotations(200)),
    kShoot(Units.degreesToRotations(180)),
    kAmp(Units.degreesToRotations(80));

    public final double angle;

    ArmPosition(double rotations) {
      this.angle = rotations;
    }
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("armAngle", Units.rotationsToDegrees(getPivotAngle()));
    // SmartDashboard.putNumber("armSetpoint", Units.rotationsToDegrees(armPosition));
    // SmartDashboard.putNumber("armError", Units.rotationsToDegrees((Math.abs(armPosition - getPivotAngle()))));
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