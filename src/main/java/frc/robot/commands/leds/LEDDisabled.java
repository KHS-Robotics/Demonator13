// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import java.sql.Driver;
import java.util.Arrays;
import java.util.Optional;
import java.util.Random;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.awt.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.NewLEDStrip;

public class LEDDisabled extends Command {
  NewLEDStrip strip;
  Random random;
  int counter;
  float speedFactor;

  /** Creates a new LEDDisabled. */
  public LEDDisabled() {
    addRequirements(RobotContainer.ledStrip);
    strip = RobotContainer.ledStrip;
    random = new Random();
    speedFactor = 1.0f;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get().equals(Alliance.Red)) {
        setPixels(Color.RED);
      } else {
        setPixels(Color.BLUE);
      }
    }
    counter++;
  }

  private void setPixels(Color c) {
    boolean allOff = true;
    for (Color color : strip.pixelArray) {
      if (strip.getHSB(color)[2] != 0) {
        allOff = false;
      }
    }

    if (allOff) {
      Arrays.fill(strip.pixelArray, c);
    } else {
      for (int i = 0; i < strip.pixelArray.length; i++) {
        if (random.nextInt() % (int) ((1f / speedFactor) * 100) == 0) {
          strip.pixelArray[i] = new Color(0);
        }
      }
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
