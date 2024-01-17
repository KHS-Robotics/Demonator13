package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Launcher extends SubsystemBase {

    private CANSparkMax launchLeader;
    private CANSparkMax launchFollower;
    private RelativeEncoder launcherEncoder;

    private SparkPIDController launchVelocityController;

    private CANSparkMax pivotMotor;
    private RelativeEncoder pivotEncoder;

    private PIDController pivotPositionController;

    private final double LAUNCHER_GEAR_RATIO = 0;
    private final double PIVOT_GEAR_RATIO = 0;

    public Launcher() {
        launchLeader = new CANSparkMax(RobotMap.LAUNCH_LEADER, MotorType.kBrushless);
        launchFollower = new CANSparkMax(RobotMap.LAUNCH_FOLLOWER, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(RobotMap.LAUNCH_PIVOT, MotorType.kBrushless);

        launchFollower.follow(launchLeader);
        launcherEncoder = launchLeader.getEncoder();
        pivotEncoder = pivotMotor.getEncoder();

        launchVelocityController = launchLeader.getPIDController();
        pivotPositionController = new PIDController(0, 0, 0);
    }

    public void setAngle() {

    }

    public void getAngle() {

    }
    
    public void setVelocity() {

    }

    public void getVelocity() {

    }

    public void calculateAngleToSpeaker() {

    }




    
    
}
