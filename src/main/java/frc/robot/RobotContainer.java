// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.shooter.SetShooterState;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.hid.OperatorStick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OldLEDStrip;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.cameras.AprilTagCamera;
import frc.robot.subsystems.cameras.NoteDetectorCamera;
import frc.robot.subsystems.drive.SwerveDrive;

public class RobotContainer {
  private static RobotContainer instance;
  public static AutoBuilder autoBuilder;

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  private static SendableChooser<Command> autoChooser;
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static final AHRS navx = new AHRS(Port.kUSB);

  /**
   * Returns the angle or "yaw" of the robot in degrees. CW positive ranging from
   * [-180, 180].
   */
  public static double getRobotYaw() {
    return navx.getYaw();
  }

  /**
   * Returns the pitch angle of the robot in degrees. This tracks the
   * forwards/backwards tilt of the robot.
   */
  public static double getRobotPitch() {
    return navx.getPitch();
  }

  /**
   * Returns the roll angle of the robot in degrees. This tracks the
   * left/right tilt of the robot.
   */
  public static double getRobotRoll() {
    return navx.getRoll();
  }

  public static final Field2d field = new Field2d();

  // Human Interface Devices (HIDs)
  public static final CommandXboxController driverController = new CommandXboxController(RobotMap.XBOX_PORT);
  public static final OperatorStick operatorStick = new OperatorStick(RobotMap.JOYSTICK_PORT);

  // Subsystems
  public static final SwerveDrive swerveDrive = new SwerveDrive();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final Arm arm = new Arm();
  // public static final AprilTagCamera frontAprilTagCamera = new
  // AprilTagCamera("FrontCamera",
  // Constants.FRONT_APRILTAG_CAMERA_OFFSET);
  // public static final AprilTagCamera rearAprilTagCamera = new
  //public static final NewLEDStrip ledStrip = new NewLEDStrip();
  public static final OldLEDStrip leds = new OldLEDStrip();

  public static final NoteDetectorCamera frontNoteCamera = new NoteDetectorCamera("NoteCamera",
      Constants.FRONT_APRILTAG_CAMERA_OFFSET);
  // AprilTagCamera("RearCamera", Constants.REAR_APRILTAG_CAMERA_OFFSET);
  public static final AprilTagCamera frontCamera = new AprilTagCamera("FrontCamera", Constants.FRONT_APRILTAG_CAMERA_OFFSET);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    this.configureSubsystemDefaultCommands();
    this.configureBindings();
    this.configureAutonmousChooser();
  }

  /** Configures the subsystem's default commands. */
  private void configureSubsystemDefaultCommands() {
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
  }

  private void configureBindings() {
    this.configureAutomatedBindings();
    this.configureXboxControllerBindings();
    this.configureOperatorStickBindings();
  }

  /** Automated bindings that happen without pressing any buttons. */
  private void configureAutomatedBindings() {

  }

  /** Binds commands to xbox controller buttons. */
  private void configureXboxControllerBindings() {

    Trigger resetOdometry = driverController.start();
    resetOdometry.onTrue(new InstantCommand(() -> swerveDrive.resetNavx()));

    Trigger slowDrive = driverController.leftTrigger(0.3);
    slowDrive.onTrue(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 0.75;
      SwerveDrive.kMaxSpeedMetersPerSecond = Math.PI / 2.0;
    }));
    slowDrive.onFalse(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
      SwerveDrive.kMaxSpeedMetersPerSecond = 4.5;
    }));


    Trigger outtake = driverController.povLeft();
    outtake.onTrue(new InstantCommand(() -> {RobotContainer.intake.outtake();}));
    outtake.onFalse(new InstantCommand(() -> RobotContainer.intake.stop()));

    Trigger intake = driverController.povRight();
    intake.onTrue(new InstantCommand(() -> {RobotContainer.intake.intake();}));
    intake.onFalse(new InstantCommand(() -> RobotContainer.intake.stop()));

    Trigger index = new Trigger(operatorStick::index);
    index.onTrue(new InstantCommand(() -> {
      RobotContainer.shooter.index();
      RobotContainer.intake.intake();
    }));
    index.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopIndexer();
      RobotContainer.intake.stop();
    }));

    Trigger outdex = new Trigger(operatorStick::outdex);
    outdex.onTrue(new InstantCommand(() -> {
      RobotContainer.shooter.outdex();
    }));
    outdex.onTrue(new InstantCommand(() -> {
      RobotContainer.shooter.stopIndexer();
    }));



    Trigger shooterDown = driverController.povDown();
    shooterDown.onTrue(new InstantCommand(() -> {
      //RobotContainer.arm.armPosition -= 0.05;
      RobotContainer.shooter.shooterAngle = 0.57;
    }));
// that puts the position of where we want the shooter wrist to go to (up)
    Trigger shooterUp = driverController.povUp();
    shooterUp.onTrue(new InstantCommand(() -> {
      //RobotContainer.arm.armPosition -= 0.05;
      RobotContainer.shooter.shooterAngle = 0.33;
    }));

    Trigger armDown = driverController.povLeft();
    armDown.onTrue(new InstantCommand(() -> {
      //RobotContainer.arm.armPosition -= 0.05;
      RobotContainer.arm.armPosition = ArmState.kStow.angle;
    }));
// that puts the position of where we want the shooter wrist to go to (up)
    Trigger armUp = driverController.povRight();
    armUp.onTrue(new InstantCommand(() -> {
      //RobotContainer.arm.armPosition -= 0.05;
      RobotContainer.arm.armPosition = ArmState.kIntake.angle;
    }));


  }

  /** Binds commands to the operator stick. */
  private void configureOperatorStickBindings() {
    Trigger shoot = new Trigger(operatorStick::shoot);
    shoot.onTrue(new InstantCommand(() -> {
      RobotContainer.shooter.setVelocity(20);
    }));
    shoot.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopShooting();
    }));

    Trigger intakeDown = new Trigger(operatorStick::intakeDown);
    intakeDown.onTrue(new InstantCommand(() -> {
      RobotContainer.intake.angleSetpoint = 0;
    }));

    Trigger intakeUp = new Trigger(operatorStick::intakeUp);
    intakeUp.onTrue(new InstantCommand(() -> {
      RobotContainer.intake.angleSetpoint = 0.44;
    }));

    Trigger intakeMid = driverController.x();
    // intakeMid.onTrue(new InstantCommand(() -> {
    // RobotContainer.intake.angleSetpoint = 0.22;
    //  }));



    Trigger armIntake = new Trigger(operatorStick::intakeSetpoint);
    armIntake.onTrue(new SetArmState(ArmState.kIntake, false));

    Trigger armAmp = new Trigger(operatorStick::ampSetpoint);
    armAmp.onTrue(new SetArmState(ArmState.kAmp, false));

    Trigger armShoot = new Trigger(operatorStick::shootSetpoint);
    armShoot.onTrue(new SetArmState(ArmState.kShoot, false));

    Trigger armStow = new Trigger(operatorStick::stowSetpoint);
    armStow.onTrue(new SetArmState(ArmState.kStow, false));



    Trigger wristIntake = new Trigger(operatorStick::wristIntake);
    wristIntake.onTrue(new SetShooterState(ShooterState.kIntake, false));

    Trigger wristShoot = new Trigger(operatorStick::wristShoot);
    wristShoot.onTrue(new SetShooterState(ShooterState.kShoot, false));
  }

  /**
   * Configures the autonomous chooser over Network Tables (e.g. Smart Dashboard).
   */
  private void configureAutonmousChooser() {
    registerNamedCommands();

    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(4.0, 0.0, 0.3),
      new PIDConstants(1.8, 0.0, 0.8),
      3.5,
      0.31592,
      new ReplanningConfig(true, true));

    AutoBuilder.configureHolonomic(
        swerveDrive::getPose,
        swerveDrive::setPose,
        swerveDrive::getChassisSpeeds,
        swerveDrive::setModuleStates,
        pathFollowerConfig,
        () -> {
          Optional<Alliance> alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        swerveDrive);

    autoBuilder = new AutoBuilder();
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("field", field);

    configureFieldLogging();
  }

  private void configureFieldLogging() {
    var logPPCurrentPoseX = new DoubleLogEntry(DataLogManager.getLog(), "/pathplanner/current/x");
    var logPPCurrentPoseY = new DoubleLogEntry(DataLogManager.getLog(), "/pathplanner/current/y");
    var logPPCurrentPoseTheta = new DoubleLogEntry(DataLogManager.getLog(), "/pathplanner/current/theta");

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);

      logPPCurrentPoseX.append(pose.getX());
      logPPCurrentPoseY.append(pose.getY());
      logPPCurrentPoseTheta.append(pose.getRotation().getDegrees());
    });

    var logPPTargetPoseX = new DoubleLogEntry(DataLogManager.getLog(), "/pathplanner/target/x");
    var logPPTargetPoseY = new DoubleLogEntry(DataLogManager.getLog(), "/pathplanner/target/y");
    var logPPTargetPoseTheta = new DoubleLogEntry(DataLogManager.getLog(), "/pathplanner/target/theta");

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);

      logPPTargetPoseX.append(pose.getX());
      logPPTargetPoseY.append(pose.getY());
      logPPTargetPoseTheta.append(pose.getRotation().getDegrees());
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("LaunchSpeaker", new ShootSpeaker());
  }
}