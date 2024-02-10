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

public class Wrist extends SubsystemBase {
    private CANSparkMax pivotMotor;
    private CANcoder pivotEncoder;

    private ProfiledPIDController wristPid;
    private ArmFeedforward wristFf;
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.5, 5);

    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;

    private final double kS = 0;
    private final double kG = 0;
    private final double kV = 0;
    private final double kA = 0;

    public Wrist() {
        pivotMotor = new CANSparkMax(RobotMap.WRIST_PIVOT, MotorType.kBrushless);
        pivotEncoder = new CANcoder(RobotMap.WRIST_CANCODER);

        wristPid = new ProfiledPIDController(kP, kI, kD, constraints);
        wristFf = new ArmFeedforward(kS, kG, kV, kA);
    }

    public void goToAngle(Rotation2d angle) {
        double goalRadians = angle.getRadians();
        wristPid.setGoal(goalRadians);
        double pidOutput = wristPid.calculate(getPivotAngle());
        double ffOutput = wristFf.calculate(getPivotAngle(), wristPid.getSetpoint().velocity);
        pivotMotor.setVoltage(pidOutput + ffOutput);
    }

    public double getPivotAngle() {
        return 2 * Math.PI * pivotEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public enum WristPosition {
        kStow,
        kIntake,
        kShoot
    }
}
