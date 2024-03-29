// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license (pizza) file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the methods
 * corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private Command autonomousRoutine;

  /**
   * This method is run when the robot is first started up and should be used for
   * any initialization code.
   */
  @Override
  public void robotInit() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Starts recording to data log
    // DataLogManager.start();
    // Record both DS control and joystick data
    // DriverStation.startDataLog(DataLogManager.getLog());

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = RobotContainer.getInstance();
    
    // for debugging
    CommandScheduler.getInstance().onCommandInitialize((command) -> {
      var cmdName = command.getName();
      System.out.println(cmdName + " started.");
    });
    CommandScheduler.getInstance().onCommandInterrupt((command) -> {
      var cmdName = command.getName();
      System.out.println(cmdName + " interrupted.");
    });
    CommandScheduler.getInstance().onCommandFinish((command) -> {
      var cmdName = command.getName();
      System.out.println(cmdName + " ended.");
    });
  }

  /**
   * This method is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic methods, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.getEntry("LED State").getInstance().flush();
    CommandScheduler.getInstance().run();
  }

  /** This method is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.shooter.stopShooting();
    RobotContainer.shooter.stopIndexer();
    RobotContainer.intake.stop();
    RobotContainer.swerveDrive.stop();
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.intake.setSetpoint(RobotContainer.intake.getPosition());
    RobotContainer.arm.setSetpoint(RobotContainer.arm.getPosition());
    // RobotContainer.shooter.setSetpoint(RobotContainer.shooter.getPosition());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousRoutine = robotContainer.getAutonomousCommand();

    // if autonomousRoutine is custom use RobotContainer.fullAuto(start, note...)
    // somehow get and parse a string from glass for start and note...
    if (autonomousRoutine.getName().equals("Dynamic")) {
      String commandString = SmartDashboard.getString("DynamicAutoString", "");
      String[] split = commandString.split(" ");
      ArrayList<String> splitList = new ArrayList<>();
      for (String s : split) {
        splitList.add(s);
      }
      ArrayList<String> notePoseStrings = new ArrayList<>();
      
      for (int i = 1; i < split.length; i++) {
        notePoseStrings.add(splitList.get(i));
      }

      String[] notePoseStringsArray = new String[notePoseStrings.size()];
      for (int i = 0; i < notePoseStrings.size(); i++) {
        notePoseStringsArray[i] = notePoseStrings.get(i);
      }

      if (commandString.equals("")) {
        autonomousRoutine = robotContainer.fullAuto("center");
      } else {
        autonomousRoutine = robotContainer.fullAuto(splitList.get(0), notePoseStringsArray);
      }

    }

    if (autonomousRoutine != null) {
      autonomousRoutine.schedule();
    }
  }

  /** This method is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousRoutine != null) {
      autonomousRoutine.cancel();
    }
  }

  @Override
  public void teleopInit() {}

  /** This method is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This method is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  /** This method is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This method is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}