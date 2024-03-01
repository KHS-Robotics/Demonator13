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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.drive.AutoPickupNote;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.SetShooterState;
import frc.robot.hid.OperatorStick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OldLEDStrip;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake.IntakeState;
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
    Trigger resetNavx = driverController.start();
    resetNavx.onTrue(new InstantCommand(() -> swerveDrive.resetNavx(), RobotContainer.swerveDrive));

    // go slow is an exception - doesn't really need to "require" the swerve drive
    Trigger slowDrive = driverController.leftTrigger(0.3);
    slowDrive.onTrue(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 0.75;
      SwerveDrive.kMaxSpeedMetersPerSecond = Math.PI / 2.0;
    }));
    // go slow is an exception - doesn't really need to "require" the swerve drive
    slowDrive.onFalse(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
      SwerveDrive.kMaxSpeedMetersPerSecond = 4.5;
    }));
  }

  /** Binds commands to the operator stick. */
  private void configureOperatorStickBindings() {
    // TODO: bind shootSpeaker to use shooting math
    Trigger shootSpeaker = new Trigger(operatorStick::scoreSpeaker);
    // shoot.onTrue(new ShootSpeaker());
    shootSpeaker.onTrue(
      new RampShooter(() -> 20)
      .andThen(
        new InstantCommand(() -> RobotContainer.shooter.feed(), RobotContainer.shooter)
      )
    );
    shootSpeaker.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopShooting();
      RobotContainer.shooter.stopIndexer();
    }, RobotContainer.shooter));

    Trigger intakeNote = new Trigger(() -> operatorStick.intakeNote() && !RobotContainer.shooter.hasNote());
    intakeNote.onTrue(new InstantCommand(() -> {
      RobotContainer.shooter.index();
      RobotContainer.intake.intake();
    }, RobotContainer.shooter, RobotContainer.intake));
    intakeNote.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopIndexer();
      RobotContainer.intake.stop();
    }, RobotContainer.shooter, RobotContainer.intake));

    Trigger outtakeNote = new Trigger(operatorStick::outtakeNote);
    outtakeNote.onTrue(new InstantCommand(() -> {
      RobotContainer.intake.outtake();
      RobotContainer.shooter.outdex();
    }, RobotContainer.intake, RobotContainer.shooter));
    outtakeNote.onFalse(new InstantCommand(() -> {
      RobotContainer.intake.stop();
      RobotContainer.shooter.stopIndexer();
    }, RobotContainer.intake, RobotContainer.shooter));

    Trigger deployIntake = new Trigger(() -> operatorStick.deployIntake() && RobotContainer.arm.isArmClearingIntake());
    deployIntake.onTrue(new SetIntakeState(IntakeState.kDown));

    Trigger retractIntake = new Trigger(() -> operatorStick.retractIntake() && RobotContainer.arm.isArmClearingIntake());
    retractIntake.onTrue(new SetIntakeState(IntakeState.kUp));
    
    Trigger intakeNoteSetpoint = new Trigger(() -> operatorStick.intakeNoteSetpoint() && RobotContainer.intake.isIntakeDown());
    intakeNoteSetpoint.onTrue(new SetShooterState(ShooterState.kIntake).alongWith(new SetArmState(ArmState.kIntake)));

    Trigger ampSetpoint = new Trigger(operatorStick::ampSetpoint);
    ampSetpoint.onTrue(new SetArmState(ArmState.kAmp).alongWith(new SetShooterState(ShooterState.kAmp)));

    Trigger shootSetpoint = new Trigger(() -> operatorStick.shootSetpoint() && RobotContainer.intake.isIntakeDown());
    shootSetpoint.onTrue(new SetArmState(ArmState.kShoot).alongWith(new SetShooterState(ShooterState.kShoot)));

    Trigger stowSetpoint = new Trigger(operatorStick::stowSetpoint);
    stowSetpoint.onTrue(new SetArmState(ArmState.kStow).alongWith(new SetShooterState(ShooterState.kIntake)));

    Trigger ampOut = new Trigger(operatorStick::scoreAmp);
    ampOut.onTrue(
      new InstantCommand(() -> RobotContainer.shooter.driveShooter(-14), RobotContainer.shooter)
      .andThen(
        new WaitCommand(0.5)
        .andThen(
          new InstantCommand(() -> RobotContainer.shooter.index(), RobotContainer.shooter)
        )
      )
    );
    ampOut.onFalse(new InstantCommand(() ->{
      RobotContainer.shooter.stopIndexer();
      RobotContainer.shooter.stopShooting();
    }, RobotContainer.shooter));
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
      Constants.DRIVE_BASE_RADIUS_METERS,
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
    NamedCommands.registerCommand("DeployIntake", new SetIntakeState(IntakeState.kDown));
    NamedCommands.registerCommand("RetractIntake", new SetIntakeState(IntakeState.kUp));
    NamedCommands.registerCommand("SetArmForScore", new SetArmState(ArmState.kShoot));
    NamedCommands.registerCommand("SetArmAndShooterForIntake", new SetShooterState(ShooterState.kIntake).alongWith(new SetArmState(ArmState.kIntake)));
    NamedCommands.registerCommand("AutoPickupNote", new AutoPickupNote().withTimeout(5));

    // TODO: this command is needed since we cannot lower the arm until the stops pop out by lifting the arm weight off them
    NamedCommands.registerCommand("LiftArmToDeployDemonHorns", new PrintCommand("!!! LiftArmToDeployDemonHorns not yet implemented !!!"));

    // TODO: create a LaunchSpeaker for auto that does not use the joystick controls on the drive train
    NamedCommands.registerCommand("LaunchSpeaker", new PrintCommand("!!! LaunchSpeaker not yet implemented !!!"));

    // TODO: get heading adjustment using vision as pass along to DoubleSupplier, or make a separate command AlignToSpeaker
    NamedCommands.registerCommand("AlignToSpeaker", new PrintCommand("!!! AlignToSpeaker not yet implemented !!!").alongWith(new RotateToAngle(() -> RobotContainer.swerveDrive.getHeading())));
  }
}
