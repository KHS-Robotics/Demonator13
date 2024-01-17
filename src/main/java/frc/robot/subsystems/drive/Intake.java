package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeMotor;
    private RelativeEncoder intakeEncoder;

    private CANSparkMax pivotMotor;
    private RelativeEncoder pivotEncoder;

    private PIDController pivotPositionController;

    private final double INTAKE_GEAR_RATIO = 0;
    private final double PIVOT_GEAR_RATIO = 0;

    public void intake() {
        intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        pivotMotor = new CANSparkMax(RobotMap.INTAKE_PIVOT, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        pivotPositionController = new PIDController(0, 0, 0);
    }

    public void outtake() {

    }

    public void raise() {

    }

    public void lower() {

    }

    public void getPivotAngle() {

    }
    
    public void setPivotAngle() {

    }
    
}
