// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import java.util.Arrays;
import java.util.Optional;
import java.util.Random;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.awt.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.NewLEDStrip;

public class LEDShoot extends Command {
  NewLEDStrip strip;
  Random random;
  float speedFactor;
  float currentPosition;
  float sections;

  /** Creates a new LEDDisabled. */
  public LEDShoot() {
    addRequirements(RobotContainer.ledStrip);
    strip = RobotContainer.ledStrip;
    random = new Random();
    speedFactor = 1.0f;
    sections = 5;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setPixels(null);
    strip.counter++;
  }

  private void setPixels(Color c) {
    speedFactor = -0.1f;
    currentPosition += speedFactor;

    for (int i = 0; i < strip.pixelArray.length; i++) {
      float adjustedPosition = (i + currentPosition) % strip.pixelArray.length;

      if (adjustedPosition < 0) {
        adjustedPosition += strip.pixelArray.length;
      }

      float j = (float) Math.abs(Math.pow(Math.sin((((Math.PI / 2) * adjustedPosition) * (0.02 * sections))), 2));

      if (j > 0.5) {
        j = 1;
      } else {
        j = 0;
      }

      strip.pixelArray[i] = Color.getHSBColor(0f, 1f, j);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
