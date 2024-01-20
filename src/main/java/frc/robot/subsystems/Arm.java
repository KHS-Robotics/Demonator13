package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase
{
    private CANSparkMax pivotMotor;
    private CANcoder pivotEncoder;
    private PIDController pivotPID;
    private final double gearRatio = 0.0;

    public Arm()
    {
        pivotMotor = new CANSparkMax(RobotMap.ARM_PIVOT, MotorType.kBrushless);
        pivotEncoder = new CANcoder(RobotMap.ARM_CANCODER);
        pivotPID = new PIDController(0, 0, 0);
    }

    public double getAngle()
    {
        return 0;
    }
    
    public void setAngle()
    {

    }
}