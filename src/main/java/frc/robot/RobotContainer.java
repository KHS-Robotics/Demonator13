// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

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
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.launcher.LaunchSpeaker;
import frc.robot.subsystems.cameras.AprilTagCamera;
import frc.robot.subsystems.drive.SwerveDrive;

public class RobotContainer {
  private static RobotContainer instance;
  public static AutoBuilder autoBuilder;
  private static SendableChooser<Command> autoChooser;

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  /*
   * public AutoRoutine getAutoRoutine() {
   * return autoChooser.getSelected();
   * }
   */

  public static final AHRS navx = new AHRS(Port.kMXP);

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
    return navx.getRoll();
  }

  public static double getRobotRoll() {
    return -navx.getPitch();
  }

  public static final Field2d field = new Field2d();

  // Human Interface Devices (HIDs)
  public static final CommandXboxController driverController = new CommandXboxController(RobotMap.XBOX_PORT);
  // public static final OperatorBox operatorBox = new
  // OperatorBox(RobotMap.SWITCHBOX_PORT);
  // public static final OperatorStick operatorStick = new
  // OperatorStick(RobotMap.JOYSTICK_PORT);

  // Subsystems
  public static final SwerveDrive swerveDrive = new SwerveDrive();
  // public static final Intake intake = new Intake();
  // public static final Launcher launcher = new Launcher();
  // public static final Arm arm = new Arm();
  public static final AprilTagCamera frontAprilTagCamera = new AprilTagCamera("FrontCamera",
      Constants.FRONT_APRILTAG_CAMERA_OFFSET);
  // public static final AprilTagCamera rearAprilTagCamera = new
  // AprilTagCamera("RearCamera", Constants.REAR_APRILTAG_CAMERA_OFFSET);

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
    this.configureOperatorBoxBindings();
    this.configureOperatorStickBindings();
  }

  /** Automated bindings that happen without pressing any buttons. */
  private void configureAutomatedBindings() {

  }

  /** Binds commands to xbox controller buttons. */
  private void configureXboxControllerBindings() {

    Trigger resetOdometry = driverController.start();
    resetOdometry.onTrue(new InstantCommand(() -> swerveDrive.resetOdometry()));

    Trigger slowDrive = driverController.leftTrigger(0.3);
    slowDrive.onTrue(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 0.75;
      SwerveDrive.kMaxSpeedMetersPerSecond = Math.PI / 2.0;
    }));
    slowDrive.onFalse(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
      SwerveDrive.kMaxSpeedMetersPerSecond = 3.5;
    }));
  }

  /** Binds commands to the operator box. */
  private void configureOperatorBoxBindings() {

  }

  /** Binds commands to the operator stick. */
  private void configureOperatorStickBindings() {

  }

  /**
   * Configures the autonomous chooser over Network Tables (e.g. Smart Dashboard).
   */
  private void configureAutonmousChooser() {
    registerNamedCommands();

    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(Constants.MAX_SPEED, 0.31592,
        new ReplanningConfig());

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
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("ShootSpeaker", new LaunchSpeaker());
    // add more here
  }
}