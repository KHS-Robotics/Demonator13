package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;

  private CANSparkMax pivotMotor;
  private AbsoluteEncoder pivotEncoder;

  // 0 degrees up, 177 degrees down
  public PIDController pivotPositionController;
  public ArmFeedforward pivotFeedforward;

  public SparkLimitSwitch intakeSensor;

  private final double kS = 0.15463;
  private final double kG = 0.59328;
  private final double kV = 0.9972;
  private final double kA = 0.025145;

  private double kP = 6;
  private double kI = 0;
  private double kD = 0;

  public double rotationSetpoint = IntakeState.kUp.rotations;

  public Intake() {
    intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeSensor = intakeMotor.getForwardLimitSwitch(Type.kNormallyClosed);

    pivotMotor = new CANSparkMax(RobotMap.INTAKE_PIVOT, MotorType.kBrushless);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(true);
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    pivotEncoder.setZeroOffset(0.206289);

    pivotPositionController = new PIDController(kP, kI, kD);
    pivotPositionController.setTolerance(1);

    pivotFeedforward = new ArmFeedforward(kS, kG, kV, kA);
  }

  public void goToSetpoint(double rotations) {
    double pidOutput = pivotPositionController.calculate(getPosition(), rotations);
    double ffOutput = pivotFeedforward.calculate(rotations, 0);
    pivotMotor.setVoltage(pidOutput + ffOutput);
  }

  public void setState(IntakeState state) {
    setSetpoint(state.rotations);
  }

  public void setSetpoint(double setpoint) {
    rotationSetpoint = setpoint;
    pivotPositionController.reset();
  }

  public void drivePivot(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public void intake() {
    intakeMotor.setVoltage(-12);
  }

  public void outtake() {
    intakeMotor.setVoltage(12);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  public double getPosition() {
    // rotations 0 down 0.44 up
    return pivotEncoder.getPosition();
  }

  public boolean isIntakeDown() {
    return rotationSetpoint < 0.04;
  }

  public boolean hasNoteInside() {
    return intakeSensor.isPressed();
  }

  public enum IntakeState {
    kUp(0.44),
    kMid(0.22),
    kDown(0);

    public final double rotations;

    IntakeState(double rotations) {
      this.rotations = rotations;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeAngle", getPosition());
    SmartDashboard.putNumber("IntakeSetpoint", rotationSetpoint);
    SmartDashboard.putNumber("IntakeError", Math.abs(getPosition() - rotationSetpoint));

    // kP = SmartDashboard.getNumber("kp", kP);
    // kI = SmartDashboard.getNumber("ki", kI);
    // kD = SmartDashboard.getNumber("kd", kD);
    // SmartDashboard.putNumber("kp", kP);
    // SmartDashboard.putNumber("ki", kI);
    // SmartDashboard.putNumber("kd", kD);
    // pivotPositionController.setPID(kP, kI, kD);

    goToSetpoint(rotationSetpoint);
  }

}
