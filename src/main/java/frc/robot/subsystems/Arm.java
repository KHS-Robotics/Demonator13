package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private CANcoder pivotEncoder;
  private final double gearRatio = 0.0;
  private ProfiledPIDController armPid;
  private ArmFeedforward armFf;
  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.5, 5);

  private final double kP = 0;
  private final double kI = 0;
  private final double kD = 0;

  private final double kS = 0;
  private final double kG = 0;
  private final double kV = 0;
  private final double kA = 0;

  public Arm() {
    pivotMotor = new CANSparkMax(RobotMap.ARM_PIVOT, MotorType.kBrushless);
    pivotEncoder = new CANcoder(RobotMap.ARM_CANCODER);

    armPid = new ProfiledPIDController(kP, kI, kD, constraints);
    armFf = new ArmFeedforward(kS, kG, kV, kA);
  }

  public void goToAngle(Rotation2d angle) {
    double goalRadians = angle.getRadians();
    armPid.setGoal(goalRadians);
    double pidOutput = armPid.calculate(getPivotAngle());
    double ffOutput = armFf.calculate(getPivotAngle(), armPid.getSetpoint().velocity);
    pivotMotor.setVoltage(pidOutput + ffOutput);
  }

  public void goToSetpoint(ArmPosition setpoint) {
    switch (setpoint) {
      case kStow:
        goToAngle(Rotation2d.fromDegrees(120));
      case kIntake:
        goToAngle(Rotation2d.fromDegrees(200));
      case kShoot:
        goToAngle(Rotation2d.fromDegrees(180));
      case kAmp:
        goToAngle(Rotation2d.fromDegrees(70));
    }
  }

  public double getPivotAngle() {
    return 2 * Math.PI * pivotEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public enum ArmPosition {
    kStow,
    kIntake,
    kShoot,
    kAmp
  }
}