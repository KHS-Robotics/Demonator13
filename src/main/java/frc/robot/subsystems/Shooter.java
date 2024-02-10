package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Function;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooterLeader;
  private CANSparkMax shooterFollower;
  private RelativeEncoder shooterEncoder;

  private SparkPIDController shooterVelocityController;

  private CANSparkMax pivotMotor;
  private RelativeEncoder pivotEncoder;

  private PIDController pivotPositionController;

  private final double SHOOTER_GEAR_RATIO = 0;
  private final double SHOOTER_WHEEL_RADIUS = 0.0508; // 4in to meters
  private final double PIVOT_GEAR_RATIO = 0;
  Function<Vector<N5>, Vector<N5>> projectileEquation;
  private final double MU = 0.5;
  private final double SPEAKER_HEIGHT = 3;

  public Shooter() {
    shooterLeader = new CANSparkMax(RobotMap.SHOOTER_LEADER, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(RobotMap.SHOOTER_FOLLOWER, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(RobotMap.SHOOTER_PIVOT, MotorType.kBrushless);

    shooterFollower.follow(shooterLeader);
    shooterEncoder = shooterLeader.getEncoder();
    pivotEncoder = pivotMotor.getEncoder();

    shooterVelocityController = shooterLeader.getPIDController();
    pivotPositionController = new PIDController(0, 0, 0);

    projectileEquation = (Vector<N5> x) -> {
      double[] in = x.getData();
      double vx = in[2];
      double vy = in[3];
      double v = Math.sqrt((vx * vx) + (vy * vy));
      double ax = -MU * vx * v;
      double ay = -9.8 - (MU * vy * v);

      Vector<N5> out = VecBuilder.fill(vx, vy, ax, ay, 0);

      return out;
    };
  }

  public Vector<N5> rkFour(Vector<N5> x) {
    Vector<N5> k_1 = projectileEquation.apply(x);
    Vector<N5> k_2 = projectileEquation.apply(x.plus(k_1.times(x.getData()[4] / 2.0)));
    Vector<N5> k_3 = projectileEquation.apply(x.plus(k_2.times(x.getData()[4] / 2.0)));
    Vector<N5> k_4 = projectileEquation.apply(x.plus(k_3.times(x.getData()[4])));
  
    Vector<N5> out = x.plus((k_1.plus(k_2.times(2)).plus(k_3.times(2)).plus(k_4)).times(x.getData()[4]));
    return out; 
  }

  public Vector<N5> propagateState(Vector<N4> x, double t, int intervals) {
    double dt = t / intervals;
    Vector<N5> state = VecBuilder.fill(x.get(0, 0), x.get(0, 1), x.get(0, 2), x.get(0, 3), dt);
    for (int i = 0; i < intervals; i++) {
      state = rkFour(state);
    }

    return state;
  }





  public void setAngle() {

  }

  public void getAngle() {

  }

  public void setVelocity() {

  }

  public void getVelocity() {

  }

  public Optional<Double> calcTrajectoryIntersectWithSpeakerHeightPlane(Vector<N4> x, double dt) {
    Vector<N5> state = VecBuilder.fill(x.get(0, 0), x.get(0, 1), x.get(0, 2), x.get(0, 3), dt);

    while (state.get(0, 3) > 0) {
      state = rkFour(state);

      if (state.get(0, 1) > SPEAKER_HEIGHT) {
        double timeAgo = (state.get(0, 1) - SPEAKER_HEIGHT) / state.get(0, 3);
        double intersectX = state.get(0, 0) - (timeAgo * state.get(0, 2));
        return Optional.of(intersectX);
      }
      
    }

    return Optional.empty();
  }

  public void calculateAngleToSpeaker() {

  }

}
