package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private RelativeEncoder intakeEncoder;

  private CANSparkMax pivotMotor;
  private CANcoder pivotEncoder;

  double motorSpeed = 0.5;

  // 0 degrees up, 177 degrees down
  public ProfiledPIDController pivotPositionController;
  public ArmFeedforward pivotFeedforward;

  public SparkLimitSwitch intakeSensor;

  private final double PIVOT_GEAR_RATIO = 0.0;

  // sane? values from last year we can tune later
  private final double kS = 0.15463;
  private final double kG = 0.59328;
  private final double kV = 0.9972;
  private final double kA = 0.025145;

  private final TrapezoidProfile.Constraints pivotConstraints = new TrapezoidProfile.Constraints(1.5, 5);

  public Intake() {
    intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();
    pivotMotor = new CANSparkMax(RobotMap.INTAKE_PIVOT, MotorType.kBrushless);
    pivotEncoder = new CANcoder(RobotMap.INTAKE_PIVOT_ENCODER);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    intakeSensor = pivotMotor.getForwardLimitSwitch(Type.kNormallyClosed);

    pivotPositionController = new ProfiledPIDController(Constants.INTAKE_PIVOT_P, Constants.INTAKE_PIVOT_I,
        Constants.DRIVE_ANGLE_D, pivotConstraints);
    pivotPositionController.setTolerance(1);

    pivotFeedforward = new ArmFeedforward(kS, kG, kV, kA);
  }

  public void goToAngle(Rotation2d angle) {
    double goalRadians = angle.getRadians();
    pivotPositionController.setGoal(goalRadians);
    double pidOutput = pivotPositionController.calculate(getPivotAngle());
    double ffOutput = pivotFeedforward.calculate(getPivotAngle(), pivotPositionController.getSetpoint().velocity);
    pivotMotor.setVoltage(pidOutput + ffOutput);
  }

  public void intake() {
    intakeMotor.set(12 * motorSpeed);
  }

  public void outtake() {
    intakeMotor.set(-12 * motorSpeed);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  public double getPivotAngle() {
    // radians 0 is up 180ish is down
    return 2 * Math.PI * pivotEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public void setPosition(IntakeSetpoint setpoint) {
    if (setpoint.equals(IntakeSetpoint.kDown)) {
      goToAngle(Rotation2d.fromDegrees(0));
    } else {
      goToAngle(Rotation2d.fromDegrees(177));
    }
  }

  public boolean hasNoteInside() {
    return intakeSensor.isPressed();
  }

  public enum IntakeSetpoint {
    kUp,
    kDown
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeAngle", getPivotAngle());
    SmartDashboard.putNumber("IntakeVelocityRPM", intakeEncoder.getVelocity());
  }

}
