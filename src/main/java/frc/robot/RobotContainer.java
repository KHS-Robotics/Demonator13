// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.drive.Indexer;
import frc.robot.subsystems.drive.Intake;
import frc.robot.subsystems.drive.Launcher;
import frc.robot.subsystems.drive.SwerveDrive;

public class RobotContainer {
  private static RobotContainer instance;

  private static Intake intake;
  private static Launcher launcher;
  private static Indexer indexer;

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  /*
  public AutoRoutine getAutoRoutine() {
    return autoChooser.getSelected();
  }
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
  public static final PowerDistribution pdp = new PowerDistribution();

  // Human Interface Devices (HIDs)
  public static final CommandXboxController driverController = new CommandXboxController(RobotMap.XBOX_PORT);
  //public static final OperatorBox operatorBox = new OperatorBox(RobotMap.SWITCHBOX_PORT);
  //public static final OperatorStick operatorStick = new OperatorStick(RobotMap.JOYSTICK_PORT);

  // Subsystems
  public static final SwerveDrive swerveDrive = new SwerveDrive();

  //public static final OldLEDStrip leds = new OldLEDStrip();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    this.configureSubsystemDefaultCommands();
    this.configureBindings();
  }

  /** Configures the subsystem's default commands. */
  private void configureSubsystemDefaultCommands() {
    //swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
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
}