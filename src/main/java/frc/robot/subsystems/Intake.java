package frc.robot.subsystems;

import org.apache.commons.math3.filter.MeasurementModel;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private RelativeEncoder intakeEncoder;

  private CANSparkMax pivotMotor;
  private AbsoluteEncoder pivotEncoder;

  // public SysIdRoutineLog log = new SysIdRoutineLog("intake log");
  // public SysIdRoutine routine = new SysIdRoutine(
  //   new SysIdRoutine.Config(),
  //   new SysIdRoutine.Mechanism(this::drivePivot, this::logMotors, null)
  // );
  

  double motorSpeed = 1;

  // 0 degrees up, 177 degrees down
  public PIDController pivotPositionController;
  public ArmFeedforward pivotFeedforward;

  public SparkLimitSwitch intakeSensor;

  private final double PIVOT_GEAR_RATIO = 0.0;

  // sane? values from last year we can tune later
  private final double kS = 0.15463;
  private final double kG = 0.59328;
  private final double kV = 0.9972;
  private final double kA = 0.025145;

  private  double kP = 6;
  private  double kI = 0;
  private  double kD = 0;

  public double angleSetpoint = 0.44;

  //private final TrapezoidProfile.Constraints pivotConstraints = new TrapezoidProfile.Constraints(1.5, 5);

  public Intake() {
    intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();
    pivotMotor = new CANSparkMax(RobotMap.INTAKE_PIVOT, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    pivotEncoder.setZeroOffset(0.206289);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(true);

    intakeSensor = intakeMotor.getForwardLimitSwitch(Type.kNormallyClosed);

    pivotPositionController = new PIDController(kP, kI, kD);
    pivotPositionController.setTolerance(1);

    pivotFeedforward = new ArmFeedforward(kS, kG, kV, kA);
  }

  public void goToAngle(double rotations) {
    //pivotPositionController.setGoal(goalRadians);
    double pidOutput = pivotPositionController.calculate(getPivotAngle(), rotations);
    double ffOutput = pivotFeedforward.calculate(rotations, 0);
    pivotMotor.setVoltage(pidOutput + ffOutput);
  }

  public void logMotors() {
  }

  public void drivePivot(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public void intake() {
    intakeMotor.setVoltage(12 * motorSpeed);
  }

  public void outtake() {
    intakeMotor.setVoltage(-12 * motorSpeed);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  public double getPivotAngle() {
    // rotations 0 down 0.44 up
    return pivotEncoder.getPosition();
  }

  public boolean hasNoteInside() {
    return intakeSensor.isPressed();
  }

  public enum IntakeState {
    kUp(0.44),
    kMid(0.22),
    kDown(0);

    public final double angle;

    IntakeState(double rotations) {
      this.angle = rotations;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeAngle", getPivotAngle());
    SmartDashboard.putNumber("IntakeState", angleSetpoint);
    SmartDashboard.putNumber("IntakeVelocityRPM", intakeEncoder.getVelocity());

    // kP = SmartDashboard.getNumber("kp", kP);
    // kI = SmartDashboard.getNumber("ki", kI);
    // kD = SmartDashboard.getNumber("kd", kD);
    // SmartDashboard.putNumber("kp", kP);
    // SmartDashboard.putNumber("ki", kI);
    // SmartDashboard.putNumber("kd", kD);
    // pivotPositionController.setPID(kP, kI, kD);

    goToAngle(angleSetpoint);
  }

}
