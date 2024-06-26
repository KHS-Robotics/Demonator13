package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

  private CANSparkMax shootMotor;
  private RelativeEncoder shooterEncoder;

  private CANSparkMax indexMotor;
  private SparkLimitSwitch indexSensor;

  private final SparkPIDController shooterPID;

  private final double kShooterP = 1;
  private final double kShooterI = 0.01;
  private final double kShooterD = 0.5;

  private final double kMaxNeoRPM = 5676;
  private final double kWheelRadius = Units.inchesToMeters(2);
  private final double kWheelCircumference = 2 * Math.PI * kWheelRadius;
  private final double kMaxSpeedMetersPerSecond = kMaxNeoRPM * kWheelCircumference;
  private final double kShooterFF = 1 / kMaxSpeedMetersPerSecond;

  public double veloctiySetpoint;
  
  public static InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  public Shooter() {
    shootMotor = new CANSparkMax(RobotMap.SHOOTER, MotorType.kBrushless);
    shootMotor.setInverted(true);

    indexMotor = new CANSparkMax(RobotMap.INDEXER, MotorType.kBrushless);
    indexMotor.setIdleMode(IdleMode.kBrake);
    indexSensor = indexMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    shootMotor.setSmartCurrentLimit(40);
    indexMotor.setSmartCurrentLimit(40);

    shooterEncoder = shootMotor.getEncoder();
    // rpm to rev/s to m/s
    shooterEncoder.setVelocityConversionFactor((1 / 60.0) * (2 * Math.PI * kWheelRadius));

    shooterPID = shootMotor.getPIDController();
    shooterPID.setP(kShooterP);
    shooterPID.setI(kShooterI);
    shooterPID.setD(kShooterD);
    shooterPID.setFF(kShooterFF);
    shooterPID.setIZone(3);
    shooterPID.setOutputRange(-1, 0);

    shooterTable.put(1.27, 0.75);
    shooterTable.put(1.47, 0.745);
    shooterTable.put(1.67, 0.74);
    shooterTable.put(1.87, 0.731);
    shooterTable.put(2.07, 0.72);
    shooterTable.put(2.27, 0.7175);
    shooterTable.put(2.47, 0.714);
    shooterTable.put(2.67, 0.713);
    shooterTable.put(2.87, 0.7076);
    shooterTable.put(2.97, 0.706);
    shooterTable.put(3.07, 0.7058);
    shooterTable.put(3.17, 0.7045);
    shooterTable.put(3.27, 0.70325);
    shooterTable.put(3.37, 0.702);
    shooterTable.put(3.47, 0.7);
    shooterTable.put(3.57, 0.698);
    
  }

  public void driveShooter(double volts) {
    shootMotor.setVoltage(volts);
    this.veloctiySetpoint = 0;
  }

  // m/s
  public void setVelocity(double velocity) {
    this.veloctiySetpoint = velocity;
    shooterPID.setReference(-velocity, ControlType.kVelocity);
  }

  // m/s
  public double getVelocity() {
    return shooterEncoder.getVelocity();
  }

  public void setShooterVoltage(double voltage) {
    shootMotor.setVoltage(voltage);
  }

  public boolean isShooterRampedUp(double tolerance) {
    return Math.abs(getVelocity() - (-veloctiySetpoint)) < tolerance;
  }

  public void stopShooting() {
    shootMotor.stopMotor();
    this.veloctiySetpoint = 0;
  }

  public void index() {
    this.indexMotor.setVoltage(3.1);
  }

  public void indexAuto() {
    this.indexMotor.setVoltage(3);
  }

  public void outdex() {
    this.indexMotor.setVoltage(-10);
  }

  public void stopIndexer() {
    this.indexMotor.setVoltage(0);
    this.indexMotor.stopMotor();
  }

  public void feed() {
    indexMotor.setVoltage(12);
  }

  public boolean hasNote() {
    return indexSensor.isPressed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter-Velocity", -getVelocity());
    SmartDashboard.putBoolean("Shooter-HasNote", hasNote());
    SmartDashboard.putNumber("shooterVelocityError", (Math.abs(getVelocity() - (-veloctiySetpoint))));
    SmartDashboard.putNumber("output", shootMotor.getAppliedOutput());
    SmartDashboard.putNumber("setpoint", shootMotor.getOutputCurrent());

    // SmartDashboard.putBoolean("shooterAtSetpoint", isShooterRampedUp(1));
    // SmartDashboard.putNumbershooterSetpoint", veloctiySetpoint);
  }
}
