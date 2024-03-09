// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Dynamic;
import frc.robot.commands.arm.SetArmState;
import frc.robot.commands.drive.AutoIntake;
import frc.robot.commands.drive.AutoPickupNote;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.intake.SetIntakeState;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.SetShooterState;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.commands.shooter.WaitForNote;
import frc.robot.hid.OperatorStick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.cameras.AprilTagCamera;
import frc.robot.subsystems.cameras.NoteDetectorCamera;
import frc.robot.subsystems.drive.SwerveDrive;

public class RobotContainer {
  private static RobotContainer instance;

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  private SendableChooser<Command> autoChooser;

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private AutoBuilder autoBuilder;

  public AutoBuilder getAutoBuilder() {
    return autoBuilder;
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
  // public static final NewLEDStrip ledStrip = new NewLEDStrip();
  public static final LEDStrip leds = new LEDStrip();

  // Cameras
  public static final NoteDetectorCamera intakeCamera = new NoteDetectorCamera("NoteCamera",
      Constants.INTAKE_NOTE_CAMERA_OFFSET);
  public static final AprilTagCamera frontAprilTagCamera = new AprilTagCamera("FrontCamera",
      Constants.FRONT_APRILTAG_CAMERA_OFFSET);
  public static final AprilTagCamera rearAprilTagCamera = new AprilTagCamera("RearCamera",
      Constants.REAR_APRILTAG_CAMERA_OFFSET);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    this.configureSubsystemDefaultCommands();
    this.configureBindings();
    this.configureAutonmous();
  }

  /** Configures the subsystem's default commands. */
  private void configureSubsystemDefaultCommands() {
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox(true));
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
    // NavX + Odometry
    var resetHeading = driverController.start().debounce(1);
    resetHeading.onTrue(new InstantCommand(() -> {
      RobotContainer.swerveDrive.resetNavx();
      var isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
      RobotContainer.swerveDrive
          .setPose(new Pose2d(8, 4, isRedAlliance ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0)));
    }, RobotContainer.swerveDrive));

    var robotRelativeDrive = driverController.rightTrigger(0.5);
    robotRelativeDrive.whileTrue(new DriveSwerveWithXbox(false));

    // Scoring
    var scoreAmp = new Trigger(
        () -> driverController.getHID().getRightBumper() && RobotContainer.arm.isAtState(ArmState.kAmp));
    scoreAmp.onTrue(new InstantCommand(() -> shooter.feed()));
    scoreAmp.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopIndexer();
      shooter.setVelocity(10);
    }, RobotContainer.shooter));

    var shootManual = new Trigger(() -> driverController.getHID().getRightBumper()
        && (arm.isAtState(ArmState.kShootFromPodium) || arm.isAtState(ArmState.kShootFromSubwoofer)));
    shootManual.onTrue(new RampShooter(() -> 15).andThen(new InstantCommand(() -> shooter.feed())));
    shootManual.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopIndexer();
      if (arm.isAtState(ArmState.kShootFromSubwoofer)) {
        shooter.setVelocity(15);
      } else {
        shooter.setVelocity(10);
      }
    }, RobotContainer.shooter));

    var intakeNote = new Trigger(
        () -> Math.abs(driverController.getHID().getLeftTriggerAxis()) > 0.9 && !RobotContainer.shooter.hasNote());
    intakeNote.onTrue(new InstantCommand(() -> {
      RobotContainer.shooter.index();
      RobotContainer.intake.intake();
    }, RobotContainer.shooter, RobotContainer.intake));
    intakeNote.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopIndexer();
      RobotContainer.intake.stop();
    }, RobotContainer.shooter, RobotContainer.intake));

    var cancelAll = driverController.back();
    cancelAll.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    var autoIntakeNote = driverController.leftBumper();
    autoIntakeNote.whileTrue(new AutoIntake());
  }

  /** Binds commands to the operator stick. */
  private void configureOperatorStickBindings() {
    var shootManual = new Trigger(operatorStick::shootManual);
    shootManual.onTrue(new RampShooter(() -> 20).andThen(new InstantCommand(() -> shooter.feed())));
    shootManual.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopIndexer();
      if (arm.isAtState(ArmState.kShootFromPodium)) {
        shooter.setVelocity(20);
      } else {
        shooter.setVelocity(10);
      }
    }, RobotContainer.shooter));

    var intakeNote = new Trigger(() -> operatorStick.intakeNote() && !RobotContainer.shooter.hasNote());
    intakeNote.onTrue(new InstantCommand(() -> {
      RobotContainer.shooter.index();
      RobotContainer.intake.intake();
    }, RobotContainer.shooter, RobotContainer.intake));
    intakeNote.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopIndexer();
      RobotContainer.intake.stop();
    }, RobotContainer.shooter, RobotContainer.intake));

    // var shootSpeaker = new Trigger(() -> driverController.getHID().getBackButton() && arm.isAtState(ArmState.kShootFromPodium) && shooter.hasNote());
    // shootSpeaker.whileTrue(new ShootSpeaker());
    // shootSpeaker.onFalse(new InstantCommand(() -> {
    //   shooter.stopIndexer();
    //   shooter.setVelocity(10);
    // }));

    var outtakeNote = new Trigger(operatorStick::outtakeNote);
    outtakeNote.onTrue(new InstantCommand(() -> {
      RobotContainer.intake.outtake();
      RobotContainer.shooter.outdex();
    }, RobotContainer.intake, RobotContainer.shooter));
    outtakeNote.onFalse(new InstantCommand(() -> {
      RobotContainer.intake.stop();
      RobotContainer.shooter.stopIndexer();
    }, RobotContainer.intake, RobotContainer.shooter));

    var deployIntake = new Trigger(() -> operatorStick.deployIntake() && RobotContainer.arm.isArmClearingIntake());
    deployIntake.onTrue(new SetIntakeState(IntakeState.kDown));

    var retractIntake = new Trigger(() -> operatorStick.retractIntake() && RobotContainer.arm.isArmClearingIntake());
    retractIntake.onTrue(new SetIntakeState(IntakeState.kUp));

    var midIntake = new Trigger(operatorStick::midIntake);
    midIntake.onTrue(new SetIntakeState(IntakeState.kMid));

    var handoffArm = new Trigger(operatorStick::handoffArm);
    var handoffArmCmd = new ConditionalCommand(
        // onTrue
        new SetIntakeState(IntakeState.kDown)
            .andThen((new SetShooterState(ShooterState.kIntake).alongWith(new SetArmState(ArmState.kIntake)))),
        // onFalse
        new SetArmState(ArmState.kStow)
            .andThen(new SetIntakeState(IntakeState.kDown)
                .andThen((new SetShooterState(ShooterState.kIntake).alongWith(new SetArmState(ArmState.kIntake))))),
        () -> arm.isArmClearingIntake() || intake.isIntakeDown());
    handoffArm.onTrue(handoffArmCmd);

    // var ampArm = new Trigger(operatorStick::ampArm);
    // ampArm.onTrue(new SetArmState(ArmState.kAmp).alongWith(new
    // SetShooterState(ShooterState.kAmp)));

    var subArm = new Trigger(operatorStick::subwooferArm);
    var subArmCmd = new ConditionalCommand(
        // onTrue
        new SetIntakeState(IntakeState.kDown)
            .andThen((new SetShooterState(ShooterState.kShootFromSubwoofer)
                .alongWith(new SetArmState(ArmState.kShootFromSubwoofer))))
            .alongWith(new InstantCommand(() -> shooter.setVelocity(15))),
        // onFalse
        new SetArmState(ArmState.kStow)
            .andThen(new SetIntakeState(IntakeState.kDown)
                .andThen((new SetShooterState(ShooterState.kShootFromSubwoofer)
                    .alongWith(new SetArmState(ArmState.kShootFromSubwoofer))))
                .alongWith(new InstantCommand(() -> shooter.setVelocity(15)))),
        () -> arm.isArmClearingIntake() || intake.isIntakeDown());
    subArm.onTrue(subArmCmd);

    // var stowArm = new Trigger(operatorStick::stowArm);
    // stowArm.onTrue(
    // (new SetArmState(ArmState.kStow).alongWith(new
    // SetShooterState(ShooterState.kIntake)))
    // .andThen(new SetIntakeState(IntakeState.kUp)));

    // UNTESTED NEW ARM CODE!!!

    Trigger stowArm = new Trigger(operatorStick::stowArm);
    stowArm.onTrue(new ProxyCommand(() -> StateCommandGenerator.goToStowCommand()));

    Trigger ampArm = new Trigger(operatorStick::ampArm);
    ampArm.onTrue(new ProxyCommand(() -> StateCommandGenerator.goToAmpCommand())
        .alongWith(new InstantCommand(() -> shooter.setVelocity(10))));

    // Trigger flatArm = new Trigger(operatorStick::podiumArm);
    // flatArm.onTrue(new ProxyCommand(() ->
    // StateCommandGenerator.goToFlatCommand()));

    // Trigger handoffArm = new Trigger(operatorStick::handoffArm);
    // handoffArm.onTrue(new ProxyCommand(() ->
    // StateCommandGenerator.goToHandoffCommand()));

    // Trigger subwooferArm = new Trigger(operatorStick::subwooferArm);
    // subwooferArm.onTrue(new ProxyCommand(() ->
    // StateCommandGenerator.goToSubwooferCommand()));

    var scoreAmp = new Trigger(operatorStick::scoreAmp);
    scoreAmp.onTrue(new InstantCommand(() -> shooter.feed()));
    scoreAmp.onFalse(new InstantCommand(() -> {
      RobotContainer.shooter.stopIndexer();
      shooter.setVelocity(10);
    }, RobotContainer.shooter));

    Trigger podiumAngle = new Trigger(operatorStick::levelArm);
    podiumAngle.onTrue(new SetArmState(ArmState.kStow).andThen(new SetIntakeState(IntakeState.kUp).andThen(
        new SetArmState(ArmState.kShootFromPodium).alongWith(new SetShooterState(ShooterState.kShootFromPodium))))
        .alongWith(new InstantCommand(() -> shooter.setVelocity(20))));

    // Trigger resetPoseWithVision = new Trigger(operatorStick::fullyTrustVision);
    // resetPoseWithVision.onTrue(new InstantCommand(() ->
    // swerveDrive.fullyTrustVision = true));
    // resetPoseWithVision.onFalse(new InstantCommand(() ->
    // swerveDrive.fullyTrustVision = false));

    Trigger rampShooter = new Trigger(operatorStick::getShooterRamping);
    rampShooter.onTrue(new InstantCommand(() -> shooter.setVelocity(10)));
    rampShooter.onFalse(new InstantCommand(() -> shooter.stopShooting()));
  }

  /**
   * Configures the autonomous chooser over Network Tables (e.g. Smart Dashboard).
   */
  private void configureAutonmous() {
    registerNamedCommands();

    var pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(4.0, 0.0, 0.3),
        new PIDConstants(1.8, 0.0, 0.8),
        SwerveDrive.kMaxSpeedMetersPerSecond,
        Constants.DRIVE_BASE_RADIUS_METERS,
        new ReplanningConfig(true, true));

    AutoBuilder.configureHolonomic(
        swerveDrive::getPose,
        swerveDrive::setPose,
        swerveDrive::getChassisSpeeds,
        swerveDrive::setModuleStates,
        pathFollowerConfig,
        () -> DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        swerveDrive);

    autoBuilder = new AutoBuilder();
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Dynamic", new Dynamic());
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putString("DynamicAutoString", "");

    configurePathPlannerLogging();
  }

  private void registerNamedCommands() {
    // Arm
    NamedCommands.registerCommand("LiftArmToDeployDemonHorns", new SetArmState(ArmState.kDeployDemonHorns));
    NamedCommands.registerCommand("SetArmAndShooterForIntake", intakeSetpointAndRun());
    NamedCommands.registerCommand("SetShootFromSubwoofer",
        new SetArmState(ArmState.kShootFromSubwoofer)
            .alongWith(new SetShooterState(ShooterState.kShootFromSubwooferAuto)));
    NamedCommands.registerCommand("SetArmForScore", new SetArmState(ArmState.kShoot));

    // Intake + Indexing
    NamedCommands.registerCommand("AutoPickupNote", new AutoPickupNote());
    NamedCommands.registerCommand("HasNote", new WaitForNote());
    NamedCommands.registerCommand("RetractIntake", new SetIntakeState(IntakeState.kUp));
    NamedCommands.registerCommand("DeployIntake", new SetIntakeState(IntakeState.kDown));
    NamedCommands.registerCommand("StartIntake", new InstantCommand(() -> {
      intake.intake();
      shooter.index();
    }, intake, shooter));
    NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {
      intake.stop();
      shooter.stopIndexer();
    }, intake, shooter));

    // Shooting
    NamedCommands.registerCommand("ShootSpeaker", new ShootSpeaker());
    NamedCommands.registerCommand("RampShooterForManualShot", new RampShooter(() -> 15));
    NamedCommands.registerCommand("Feed", feedCommand());
    NamedCommands.registerCommand("StopShooter", new InstantCommand(() -> shooter.stopShooting(), shooter));

    // Swerves
    NamedCommands.registerCommand("StopSwerves", new InstantCommand(() -> swerveDrive.stop(), swerveDrive));
    // SwerveStraighten - used when lowering intake and deploying demon horns to
    // make initial odometry updates more accurate
    NamedCommands.registerCommand("SwerveStraighten", straightenSwervesCommand());

    // General
    NamedCommands.registerCommand("ShootSubwooferSequence", shootSubwooferSequence());
    NamedCommands.registerCommand("InitSequence", initSequence());

  }

  private Command stopAllCommand() {
    return new InstantCommand(() -> {
      shooter.stopIndexer();
      shooter.stopShooting();
      intake.stop();
      swerveDrive.stop();
    }, shooter, intake, swerveDrive);
  }

  private Command feedCommand() {
    return new InstantCommand(() -> shooter.feed(), shooter)
        .andThen(new WaitCommand(0.05))
        .andThen(new InstantCommand(() -> shooter.stopIndexer(), shooter));
  }

  private Command straightenSwervesCommand() {
    return new RepeatCommand(
        new InstantCommand(() -> swerveDrive.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0))
        }),
            swerveDrive))
        .withTimeout(1)
        .andThen(new InstantCommand(() -> swerveDrive.stop(), swerveDrive));
  }

  private Command intakeSetpointAndRun() {
    return (new SetShooterState(ShooterState.kIntake)
        .alongWith(
            new SetArmState(ArmState.kIntake))
        .andThen(
            new InstantCommand(() -> {
              intake.intake();
              shooter.index();
            }, intake, shooter)));
  }

  private Command initSequence() {
    return new ParallelCommandGroup(new SetIntakeState(IntakeState.kDown), new SetArmState(ArmState.kDeployDemonHorns),
        new RampShooter(() -> 15), straightenSwervesCommand());
  }

  private Command shootSubwooferSequence() {
    return new SequentialCommandGroup(new SetShooterState(ShooterState.kShootFromSubwooferAuto)
        .alongWith(new SetArmState(ArmState.kShootFromSubwoofer)), feedCommand(), intakeSetpointAndRun());
  }

  private Command getPathCommand(String name) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(name);
    return AutoBuilder.followPath(path);
  }

  // this will get a string or something from glass idk yet
  private Command getNoteSequence(String startPosition, String notePosition) {
    String getNotePathName = startPosition + " get " + notePosition;
    String getReturnPathName = "return " + startPosition + " get " + notePosition;
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
        getPathCommand(getNotePathName).andThen(new InstantCommand(() -> swerveDrive.stop(), swerveDrive)),
        new WaitForNote().withTimeout(3)),
        getPathCommand(getReturnPathName), 
        new InstantCommand(() -> swerveDrive.stop(), swerveDrive),
        shootSubwooferSequence());
  }

  // THIS EXPECTS YOU TO PUT THE NOTE POSITIONS IN THE RIGHT ORDER!! (I don't want to sort them based on startPosition because it's 3am and I'm sleepy)
  public Command fullAuto(String startPosition, String... notePositions) {
    // if no note positions are passed shoot in place and stop everything
    if (notePositions.length == 0) {
      return initSequence().andThen(shootSubwooferSequence()).andThen(stopAllCommand());
    }

    // set all to lowercase to match the path names
    for (String s : notePositions) {
      s = s.toLowerCase();
    }
    startPosition = startPosition.toLowerCase();

    // get each note sequence, each noteSequence includes path to get, grab, if missed timeout after 3 secs, return path, shoot
    Command[] noteSequences = new Command[notePositions.length];
    for (int i = 0; i < notePositions.length; i++) {
      System.out.println(notePositions[i]);
      noteSequences[i] = getNoteSequence(startPosition, notePositions[i]);
    }

    // combine init and all noteSequence
    System.out.println(startPosition + notePositions);
    return new SequentialCommandGroup(initSequence(), new SequentialCommandGroup(noteSequences));
  }

  private void configurePathPlannerLogging() {
    SmartDashboard.putData("field", field);

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
}
