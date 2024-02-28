package frc.robot.subsystems;

import java.util.function.Function;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.MaxIter;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.SimpleValueChecker;
import org.apache.commons.math3.optim.linear.NonNegativeConstraint;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.MultiDirectionalSimplex;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

  private CANSparkMax shootMotor;
  private RelativeEncoder shooterEncoder;

  private CANSparkMax pivotMotor;
  private AbsoluteEncoder pivotEncoder;

  private ArmFeedforward pivotFF;
  private PIDController pivotPID;

  private boolean hasNote = true;

  private CANSparkMax indexMotor;
  private SparkLimitSwitch indexSensor;

  Function<double[], double[]> projectileEquation3d;

  private double indexSpeed = 0.5;

  private final double DRAG_COEFFICIENT = 0.5;
  private final double AIR_DENSITY = 1.225;
  private final double CROSS_SECTIONAL_AREA = 0.018;
  private final double NOTE_MASS = 0.2353;
  private final double MU = (DRAG_COEFFICIENT * AIR_DENSITY * CROSS_SECTIONAL_AREA) / (2 * NOTE_MASS);
  private final SparkPIDController shooterPID;

  private final double SHOOTER_PIVOT_TO_END = 0.37516;
  private final Translation3d SHOOTER_PIVOT_ROBOT_REL = new Translation3d(-0.2757, 0, 0.5972);

  private final double v0 = 20;

  private final double kShooterP = 1;
  private final double kShooterI = 0.01;
  private final double kShooterD = 0.5;

  private final double pivotkS = 0.15463;
  private final double pivotkG = 0.30328;
  private final double pivotkV = 0.9972;
  private final double pivotkA = 0.025145;
  private final double pivotkP = 35;
  private final double pivotkI = 0;
  private final double pivotkD = 2;

  private final double kMaxNeoRPM = 5676;
  private final double kWheelRadius = Units.inchesToMeters(2);
  private final double kWheelCircumference = 2 * Math.PI * kWheelRadius;
  private final double kMaxSpeedMetersPerSecond = kMaxNeoRPM * kWheelCircumference;
  private final double kShooterFF = 1 / kMaxSpeedMetersPerSecond;

  public double veloctiySetpoint;
  public double shooterAngle = 0;

  public Shooter() {
    shootMotor = new CANSparkMax(RobotMap.SHOOTER, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(RobotMap.SHOOTER_PIVOT, MotorType.kBrushless);
    indexMotor = new CANSparkMax(RobotMap.INDEXER, MotorType.kBrushless);
    indexMotor.setIdleMode(IdleMode.kCoast);
    indexSensor = indexMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    shootMotor.setInverted(true);

    shooterEncoder = shootMotor.getEncoder();
    // rpm to rev/s to m/s
    shooterEncoder.setVelocityConversionFactor((1 / 60.0) * (2 * Math.PI * kWheelRadius));
    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setZeroOffset(0);

    shooterPID = shootMotor.getPIDController();
    shooterPID.setP(kShooterP);
    shooterPID.setI(kShooterI);
    shooterPID.setD(kShooterD);
    shooterPID.setFF(kShooterFF);
    shooterPID.setIZone(3);
    shooterPID.setOutputRange(-1, 0);

    pivotFF = new ArmFeedforward(pivotkS, pivotkG, pivotkV, pivotkA);
    pivotPID = new PIDController(pivotkP, pivotkI, pivotkD);

    projectileEquation3d = (double[] x) -> {
      double vx = x[3];
      double vy = x[4];
      double vz = x[5];
      double v = Math.sqrt((vx * vx) + (vy * vy) + (vz * vz));
      double ax = -MU * vx * v;
      double ay = -MU * vy * v;
      double az = -9.8 - (MU * vz * v);

      return new double[] { vx, vy, vz, ax, ay, az, 0, 0 };
    };
  }

  public void goToAngle(double angle) {
    double pidOutput = pivotPID.calculate(getPivotAngle(), shooterAngle);
    double ffOutput = pivotFF.calculate(getPivotAngle() + RobotContainer.arm.getPivotAngle(), 0);
    var output = pidOutput + ffOutput;
    pivotMotor.setVoltage(-output);
  }

  public double getAbsoluteAngle() {
    return getPivotAngle() + (RobotContainer.arm.getPivotAngle() - 0.5);
  }

  public void goToSetpoint(ShooterState setpoint) {
    goToAngle(setpoint.angle);
  }

  public void driveShooter(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public double getPivotAngle() {
    return pivotEncoder.getPosition();
  }

  // m/s
  public void setVelocity(double velocity) {
    shooterPID.setReference(-velocity, ControlType.kVelocity);
  }

  // m/s
  public double getVelocity() {
    return shooterEncoder.getVelocity();
  }

  public void setSetpoint(ShooterState setpoint) {
    this.shooterAngle = setpoint.angle;
    pivotPID.reset();
  }

  public boolean isShooterAtSetpoint() {
    return Math.abs(getVelocity() - veloctiySetpoint) < 5;
  }

  public void stopShooting() {
    shootMotor.stopMotor();
  }

  public boolean atAngleSetpoint(double tolerance) {
    pivotPID.setTolerance(tolerance);
    return pivotPID.atSetpoint();
  }

  public void index() {
    this.indexMotor.setVoltage(4.5);
  }

  public void outdex() {
    this.indexMotor.setVoltage(-6);
  }

  public void stopIndexer() {
    this.indexMotor.setVoltage(0);
    this.indexMotor.stopMotor();
  }

  public void feed() {
    indexMotor.setVoltage(12);
  }

  public double[] rkFour(double[] x, Function<double[], double[]> f) {
    double h = x[x.length - 1];

    double[] k_1 = f.apply(x);
    double[] k_2 = f.apply(addVectors(x, multiplyByScalar(k_1, h / 2)));
    double[] k_3 = f.apply(addVectors(x, multiplyByScalar(k_2, h / 2)));
    double[] k_4 = f.apply(addVectors(x, multiplyByScalar(k_3, h)));

    double[] out = addVectors(
        multiplyByScalar(
            addVectors(addVectors(addVectors(k_1, multiplyByScalar(k_2, 2)), multiplyByScalar(k_3, 2)), k_4), h / 6),
        x);
    out[x.length - 2] += h;
    return out;
  }

  public double[] addVectors(double[] a, double[] b) {
    double[] out = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      out[i] = a[i] + b[i];
    }
    return out;
  }

  public double[] multiplyByScalar(double[] a, double b) {
    double[] out = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      out[i] = a[i] * b;
    }
    return out;
  }

  public double[][] propagateWholeTrajectory3d(double[] k, double t, int intervals) {
    double x = k[0];
    double y = k[1];
    double z = k[2];
    double vx = k[3];
    double vy = k[4];
    double vz = k[5];

    double[][] out = new double[intervals][k.length + 2];
    double dt = t / intervals;

    double[] state = { x, y, z, vx, vy, vz, 0, dt };

    for (int i = 0; i < intervals; i++) {
      state = rkFour(state, projectileEquation3d);
      out[i] = state;
    }

    return out;
  }

  public Translation3d shooterExitRobotRelative(double theta) {
    double tx = SHOOTER_PIVOT_TO_END * Math.cos(theta);
    double ty = 0;
    double tz = SHOOTER_PIVOT_TO_END * Math.sin(theta);

    return SHOOTER_PIVOT_ROBOT_REL.plus(new Translation3d(tx, ty, tz));
  }

  public Translation3d shooterExitFieldRelative(Pose2d robotPose, Translation3d shooterRobotRelative) {
    double diffX = shooterRobotRelative.getX();

    Translation3d shooterFieldRelative = new Translation3d(
        robotPose.getX() + (diffX * robotPose.getRotation().getCos()),
        robotPose.getY() + (diffX * robotPose.getRotation().getSin()), shooterRobotRelative.getZ());
    return shooterFieldRelative;
  }

  public Translation2d translationToSpeaker() {
    Pose2d robotPose = RobotContainer.swerveDrive.getPose();
    Translation2d speakerPose = new Translation2d(0, 5.5528);
    return speakerPose.minus(robotPose.getTranslation());
  }

  public double speakerOpeningAngleVertical() {
    Translation2d translationToSpeaker = translationToSpeaker();
    Translation2d translationToOutsideSpeaker = translationToSpeaker.plus(new Translation2d(0.46, 0));

    double speakerInsideHeight = 1.98;
    double speakerOutsideHeight = 2.11;

    double theta1 = Math.atan2(speakerInsideHeight, translationToSpeaker.getX());
    double theta2 = Math.atan2(speakerOutsideHeight, translationToOutsideSpeaker.getX());
    return theta2 - theta1;
  }

  public double[] optimizeShooterOrientation(double initialTheta, double initialPhi, double initialTime,
      double targetX, double targetY, double targetZ) {

    MultivariateFunction f = point -> {
      // calculate trajectory given theta phi and t
      Pose2d pose = RobotContainer.swerveDrive.getPose();
      Translation3d shooterPose = shooterExitFieldRelative(pose, shooterExitRobotRelative(initialPhi));

      double[] in = { shooterPose.getX(), shooterPose.getY(), shooterPose.getZ(),
          RobotContainer.swerveDrive.vX + (v0 * Math.sin(Math.PI / 2 - point[0]) * Math.cos(point[1])),
          RobotContainer.swerveDrive.vY + (v0 * Math.sin(Math.PI / 2 - point[0]) * Math.sin(point[1])),
          v0 * Math.cos(Math.PI / 2 - point[0]) };
      double[][] trajectory = propagateWholeTrajectory3d(in, point[2], 1);
      double[] finalPosition = trajectory[trajectory.length - 1];

      double xdiff = targetX - finalPosition[0];
      double ydiff = targetY - finalPosition[1];
      double zdiff = targetZ - finalPosition[2];

      double distance = Math.sqrt((xdiff * xdiff) + (ydiff * ydiff) + (zdiff * zdiff));

      return distance;
    };

    ObjectiveFunction objective = new ObjectiveFunction(f);

    double[] initialGuess = new double[] { initialTheta, initialPhi, initialTime };
    InitialGuess guess = new InitialGuess(initialGuess);
    MaxIter maxIter = new MaxIter(20000);
    // look at my lawyer dawg I'm goin to jail!!!
    MaxEval maxEval = new MaxEval(100000);
    MultiDirectionalSimplex simplex = new MultiDirectionalSimplex(3, 0.001);
    SimplexOptimizer optimizer = new SimplexOptimizer(new SimpleValueChecker(0.000001, 0.000001));

    NonNegativeConstraint constraint = new NonNegativeConstraint(true);

    PointValuePair optimum = optimizer.optimize(
        maxIter,
        maxEval,
        objective,
        GoalType.MINIMIZE,
        guess,
        simplex,
        constraint

    );

    return optimum.getPoint();
  }

  public boolean hasNote() {
    return indexSensor.isPressed();
  }

  public enum ShooterState {
    kIntake(0.3),
    kShoot(0.2),
    kAmp(0.1);

    public final double angle;

    ShooterState(double rotations) {
      this.angle = rotations;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooterAngle", getPivotAngle());
    SmartDashboard.putNumber("shooterAngleSetpoint", shooterAngle);
    SmartDashboard.putNumber("shooterAngleError", Math.abs(shooterAngle - getPivotAngle()));
    SmartDashboard.putNumber("shooterAngleAbsolute", getAbsoluteAngle());
    SmartDashboard.putNumber("kG", pivotkG);
    SmartDashboard.putNumber("Shooter-Velocity", getVelocity());
    SmartDashboard.putBoolean("Shooter-HasNote", hasNote());


    goToAngle(shooterAngle);
  }
}
