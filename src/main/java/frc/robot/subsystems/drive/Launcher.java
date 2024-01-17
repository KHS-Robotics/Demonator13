package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

    private CANSparkMax launchLeader;
    private CANSparkMax launchFollower;

    private CANSparkMax pivotMotor;

    public Launcher() {
        launchFollower.follow(launchLeader);
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
