/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static final int XBOX_PORT = 0;
  public static final int SWITCHBOX_PORT = 1;
  public static final int JOYSTICK_PORT = 2;

  public static final int INTAKE_MOTOR = 0;
  public static final int INTAKE_PIVOT = 0;

  public static final int SHOOTER_LEADER = 0;
  public static final int SHOOTER_FOLLOWER = 0;
  public static final int SHOOTER_PIVOT = 0;

  public static final int ARM_PIVOT = 0;
  public static final int ARM_CANCODER = 0;

  public static final int FRONT_LEFT_PIVOT = 10;
  public static final int FRONT_RIGHT_PIVOT = 20;
  public static final int REAR_LEFT_PIVOT = 30;
  public static final int REAR_RIGHT_PIVOT = 40;

  public static final int FRONT_LEFT_DRIVE = 11;
  public static final int FRONT_RIGHT_DRIVE = 21;
  public static final int REAR_LEFT_DRIVE = 31;
  public static final int REAR_RIGHT_DRIVE = 41;

  public static final int FRONT_LEFT_PIVOT_ENCODER = 12;
  public static final int FRONT_RIGHT_PIVOT_ENCODER = 22;
  public static final int REAR_LEFT_PIVOT_ENCODER = 32;
  public static final int REAR_RIGHT_PIVOT_ENCODER = 42;
}