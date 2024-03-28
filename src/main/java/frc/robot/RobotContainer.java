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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
import frc.robot.commands.shooter.NoteVisible;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.commands.shooter.WaitForNote;
import frc.robot.hid.OperatorStick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Intake.IntakeState;
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
  public static final Climber climber = new Climber();
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
    var resetHeading = driverController.start().debounce(0.25);
    resetHeading.onTrue(new InstantCommand(() -> {
      swerveDrive.resetNavx();
      var isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
      swerveDrive.setPose(new Pose2d(8, 4, isRedAlliance ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0)));
    }, swerveDrive));

    var robotRelativeDrive = driverController.rightTrigger(0.5);
    robotRelativeDrive.whileTrue(new DriveSwerveWithXbox(false));

    var slowDrive = driverController.x();
    slowDrive.onTrue(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 1;
      SwerveDrive.kMaxSpeedMetersPerSecond = 1;
    }));
    slowDrive.onFalse(new InstantCommand(() -> {
      SwerveDrive.kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
      SwerveDrive.kMaxSpeedMetersPerSecond = 4.6;
    }));

    var scoreAmp = new Trigger(
        () -> driverController.getHID().getRightBumper() && -shooter.getVelocity() > 8 && arm.isAtState(ArmState.kAmp));
    scoreAmp.onTrue(new InstantCommand(() -> shooter.feed()));
    scoreAmp.onFalse(new WaitCommand(1).andThen(new InstantCommand(() -> {
      shooter.stopIndexer();
      shooter.setVelocity(10);
    }, shooter)));

    var shootManual = new Trigger(() -> driverController.getHID().getRightBumper() && -shooter.getVelocity() > 8
        && !arm.isAtState(ArmState.kAmp));
    shootManual.onTrue(new InstantCommand(() -> shooter.feed()));
    shootManual.onFalse(new WaitCommand(1).andThen(new InstantCommand(() -> {
      shooter.stopIndexer();
      if (!arm.isAtState(ArmState.kAmp)) {
        shooter.setVelocity(17);
      } else {
        shooter.setVelocity(10);
      }
    }, shooter)));

    var intakeNote = new Trigger(
        () -> Math.abs(driverController.getHID().getLeftTriggerAxis()) > 0.9 && !shooter.hasNote()
            && arm.isAtState(ArmState.kIntake));
    intakeNote.onTrue(new InstantCommand(() -> {
      shooter.index();
      intake.intake();
    }, shooter, intake));
    intakeNote.onFalse(new InstantCommand(() -> {
      shooter.stopIndexer();
      intake.stop();
    }, shooter, intake));

    var cancelAll = driverController.back();
    cancelAll.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    var autoIntakeNote = new Trigger(
        () -> driverController.getHID().getLeftBumper() && arm.isAtState(ArmState.kIntake) && !shooter.hasNote());
    autoIntakeNote.whileTrue(new AutoIntake());

    var shootSpeaker = new Trigger(() -> driverController.getHID().getAButton() && shooter.hasNote());
    shootSpeaker.whileTrue(new ShootSpeaker());
    shootSpeaker.onFalse(new InstantCommand(() -> {
      // shooter.setVelocity(15);
    }));
  }

  /** Binds commands to the operator stick. */
  private void configureOperatorStickBindings() {
    var shootManual = new Trigger(() -> operatorStick.shootManual() && -shooter.getVelocity() > 8);
    shootManual.onTrue(new RampShooter(() -> 20).andThen(new InstantCommand(() -> shooter.feed())));
    shootManual.onFalse(new WaitCommand(1).andThen(new InstantCommand(() -> {
      shooter.stopIndexer();
      if (arm.isAtState(ArmState.kShootFromPodium)) {
        shooter.setVelocity(20);
      } else {
        shooter.setVelocity(10);
      }
    }, shooter)));

    var intakeNote = new Trigger(
        () -> operatorStick.intakeNote() && !shooter.hasNote() && arm.isAtState(ArmState.kIntake));
    intakeNote.onTrue(new InstantCommand(() -> {
      shooter.index();
      intake.intake();
    }, shooter, intake));
    intakeNote.onFalse(new InstantCommand(() -> {
      shooter.stopIndexer();
      intake.stop();
    }, shooter, intake));

    var outtakeNote = new Trigger(operatorStick::outtakeNote);
    outtakeNote.onTrue(new InstantCommand(() -> {
      intake.outtake();
      shooter.outdex();
      shooter.setShooterVoltage(-12);
    }, intake, shooter));
    outtakeNote.onFalse(new InstantCommand(() -> {
      intake.stop();
      shooter.stopIndexer();
      shooter.stopShooting();
    }, intake, shooter));

    var deployIntake = new Trigger(() -> operatorStick.deployIntake() && arm.isArmClearingIntake());
    deployIntake.onTrue(new SetIntakeState(IntakeState.kDown));

    var retractIntake = new Trigger(() -> operatorStick.retractIntake() && arm.isArmClearingIntake());
    retractIntake.onTrue(new SetIntakeState(IntakeState.kUp));

    var handoffArm = new Trigger(operatorStick::handoffArm);
    var handoffArmCmd = new ConditionalCommand(
        // onTrue
        new SetIntakeState(IntakeState.kDown)
            .andThen(new SetArmState(ArmState.kIntake)),
        // onFalse
        new SetArmState(ArmState.kStow)
            .andThen(new SetIntakeState(IntakeState.kDown)
                .andThen(new SetArmState(ArmState.kIntake))),
        () -> arm.isArmClearingIntake() || intake.isIntakeDown());
    handoffArm.onTrue(handoffArmCmd);

    var subArm = new Trigger(() -> operatorStick.subwooferArm() && (shooter.hasNote() || !intake.hasNoteInside()));
    var subArmCmd = new ConditionalCommand(
        // onTrue
        new SetIntakeState(IntakeState.kDown)
            .andThen((new SetArmState(ArmState.kShootFromSubwoofer)))
            .alongWith(new InstantCommand(() -> shooter.setVelocity(15))),
        // onFalse
        new SetArmState(ArmState.kStow)
            .andThen(new SetIntakeState(IntakeState.kDown)
                .andThen((new SetArmState(ArmState.kShootFromSubwoofer)))
                .alongWith(new InstantCommand(() -> shooter.setVelocity(15)))),
        () -> arm.isArmClearingIntake() || intake.isIntakeDown());
    subArm.onTrue(subArmCmd);

    var stowArm = new Trigger(() -> operatorStick.stowArm() && (shooter.hasNote() || !intake.hasNoteInside()));
    stowArm.onTrue(new ProxyCommand(() -> StateCommandGenerator.goToStowCommand()));

    var ampArm = new Trigger(() -> operatorStick.ampArm() && (shooter.hasNote() || !intake.hasNoteInside()));
    ampArm.onTrue(new ProxyCommand(() -> StateCommandGenerator.goToAmpCommand())
        .alongWith(new InstantCommand(() -> shooter.setVelocity(10))));

    var scoreAmp = new Trigger(() -> operatorStick.scoreAmp() && -shooter.getVelocity() > 8);
    scoreAmp.onTrue(new InstantCommand(() -> shooter.feed()));
    scoreAmp.onFalse(new WaitCommand(1).andThen(
        new InstantCommand(() -> {
          shooter.stopIndexer();
          shooter.setVelocity(10);
        }, shooter)));

    var podiumAngle = new Trigger(
        () -> operatorStick.podiumArm() && (shooter.hasNote() || !intake.hasNoteInside()));
    podiumAngle.onTrue(new SetArmState(ArmState.kStow).andThen(new SetIntakeState(IntakeState.kUp).andThen(
        new SetArmState(ArmState.kShootFromPodium)))
        .alongWith(new InstantCommand(() -> shooter.setVelocity(20))));

    var rampShooter = new Trigger(operatorStick::getShooterRamping);
    rampShooter.onTrue(new InstantCommand(() -> shooter.setVelocity(10)));
    rampShooter.onFalse(new InstantCommand(() -> shooter.stopShooting()));

    var feedSetpoint = new Trigger(
        () -> operatorStick.feedShotSetpoint() && (shooter.hasNote() || !intake.hasNoteInside()));
    feedSetpoint.onTrue(new SetIntakeState(IntakeState.kDown).andThen(
        new SetArmState(ArmState.kFeedFromCenter)
            .alongWith(new InstantCommand(() -> shooter.setVelocity(16)))));

    var raiseRight = new Trigger(operatorStick::raiseRight);
    raiseRight.onTrue(new InstantCommand(() -> climber.raiseRight(), climber));
    raiseRight.onFalse(new InstantCommand(() -> climber.stopRight(), climber));

    var raiseLeft = new Trigger(operatorStick::raiseLeft);
    raiseLeft.onTrue(new InstantCommand(() -> climber.raiseLeft(), climber));
    raiseLeft.onFalse(new InstantCommand(() -> climber.stopLeft(), climber));

    var lowerRight = new Trigger(operatorStick::lowerRight);
    lowerRight.onTrue(new InstantCommand(() -> climber.lowerRight(), climber));
    lowerRight.onFalse(new InstantCommand(() -> climber.stopRight(), climber));

    var lowerLeft = new Trigger(operatorStick::lowerLeft);
    lowerLeft.onTrue(new InstantCommand(() -> climber.lowerLeft(), climber));
    lowerLeft.onFalse(new InstantCommand(() -> climber.stopLeft(), climber));

    var stepAngleUp = new Trigger(operatorStick::stepAngleUp);
    stepAngleUp.onTrue(new InstantCommand(() -> arm.setSetpoint(arm.rotationSetpoint + 0.0025)));

    var stepAngleDown = new Trigger(operatorStick::stepAngleDown);
    stepAngleDown.onTrue(new InstantCommand(() -> arm.setSetpoint(arm.rotationSetpoint - 0.0025)));
  }

  /**
   * Configures the autonomous chooser over Network Tables (e.g. Smart Dashboard).
   */
  private void configureAutonmous() {
    registerNamedCommands();

    var pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(4.0, 0.0, 0.3),
        new PIDConstants(1.5, 0.0, 0.8),
        SwerveDrive.kMaxSpeedMetersPerSecond,
        Constants.DRIVE_BASE_RADIUS_METERS,
        new ReplanningConfig(true, false));

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
    NamedCommands.registerCommand("SetShootFromSubwoofer", new SetArmState(ArmState.kShootFromSubwooferAuto));
    NamedCommands.registerCommand("SetArmForScore", new SetArmState(ArmState.kShoot).andThen(new WaitCommand(0.2)));

    // Intake + Indexing
    NamedCommands.registerCommand("AutoIntake", new AutoIntake());
    NamedCommands.registerCommand("AutoPickupNote", new AutoPickupNote());
    NamedCommands.registerCommand("NoteVisible", new NoteVisible());
    NamedCommands.registerCommand("HasNote", new WaitForNote().withTimeout(3));
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
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(() -> shooter.stopIndexer(), shooter));
  }

  private Command straightenSwervesCommand() {
    return new RepeatCommand(
        new InstantCommand(() -> swerveDrive.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(0))
        }), swerveDrive))
        .withTimeout(1)
        .andThen(new InstantCommand(() -> swerveDrive.stop(), swerveDrive));
  }

  private Command intakeSetpointAndRun() {
    return (

    new SetArmState(ArmState.kIntake)
        .andThen(
            new InstantCommand(() -> {
              intake.intake();
              shooter.indexAuto();
            }, intake, shooter)));
  }

  private Command initSequence() {
    return new ParallelDeadlineGroup(new SetIntakeState(IntakeState.kDown),
        new SetArmState(ArmState.kDeployDemonHorns).alongWith(
            new RampShooter(() -> 15)).alongWith(straightenSwervesCommand()));
  }

  private Command shootSubwooferSequence() {
    return new SequentialCommandGroup(new RampShooter(() -> 15)
        .alongWith(new SetArmState(ArmState.kShootFromSubwooferAuto)), new WaitCommand(0.1), feedCommand(),
        intakeSetpointAndRun());
  }

  private Command getPathCommand(String name) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(name);
    return AutoBuilder.followPath(path);
  }

  private PathPlannerPath getPath(String name) {
    return PathPlannerPath.fromPathFile(name);
  }

  private Pose2d pathStartPose(String name) {
    return getPath(name).getPreviewStartingHolonomicPose();
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

  // THIS EXPECTS YOU TO PUT THE NOTE POSITIONS IN THE RIGHT ORDER!! (I don't want
  // to sort them based on startPosition because it's 3am and I'm sleepy)
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

    // get each note sequence, each noteSequence includes path to get, grab, if
    // missed timeout after 3 secs, return path, shoot
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
