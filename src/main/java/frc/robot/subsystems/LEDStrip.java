package frc.robot.subsystems;

import java.awt.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm.ArmState;

public class LEDStrip {
  Thread t;
  AddressableLED strip;
  AddressableLED strip2;
  AddressableLEDBuffer buffer;
  int numberSections;
  int counter;
  int ticksPerSecond = 50;

  private LEDState state = LEDState.kDisabled;
  private final Notifier updateLedState = new Notifier(this::updateState);

  float speedFactor = 1f;
  float sections = 5f;
  float currentPosition = 0f;
  Color[] pixelArray;

  public LEDStrip() {
    pixelArray = new Color[Constants.LED_LENGTH];
    for (int i = 0; i < pixelArray.length; i++) {
      pixelArray[i] = new Color(0);
    }

    // multithreading this has got to be genius
    t = new Thread(() -> {
      long lastTime = System.nanoTime();
      double delta = 0;
      // very accurate loop, is this bad for performance?
      while (!Thread.interrupted()) {
        double ns = 1000000000 / (double) ticksPerSecond;
        long now = System.nanoTime();
        delta += (now - lastTime) / ns;
        lastTime = now;
        if (delta >= 1) {
          update();
          delta--;
        }
      }
    });
    strip = new AddressableLED(RobotMap.LED_PORT);
    // strip2 = new AddressableLED(RobotMap.LED_PORT_2);
    strip.setLength(Constants.LED_LENGTH);
    // strip2.setLength(Constants.LED_LENGTH);
    buffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGB(i, 255, 255, 255);
    }
    strip.setData(buffer);
    // strip2.setData(buffer);
    strip.start();
    // strip2.start();

    this.numberSections = Constants.LED_LENGTH;
    t.start();
    updateLedState.startPeriodic(0.2);
  }

  public void setRGB(int index, int r, int g, int b) {
    buffer.setRGB(index, r, g, b);
  }

  public void setHSV(int index, int h, int s, int v) {
    buffer.setHSV(index, h, s, v);
  }

  public void fillBuffer(Color[] colors) {
    for (int i = 0; i < colors.length; i++) {
      setRGB(i, colors[i].getRed(), colors[i].getGreen(), colors[i].getBlue());
    }
  }

  public void setPixelColorHSB(int i, float h, float s, float b) {
    Color c = Color.getHSBColor(h, s, b);
    setRGB(i, c.getRed(), c.getGreen(), c.getBlue());
  }

  public float[] getHSB(Color c) {
    float[] rgb = new float[3];
    rgb = c.getRGBColorComponents(rgb);

    rgb[0] /= 255f;
    rgb[1] /= 255f;
    rgb[2] /= 255f;

    float cmax = Math.max(rgb[0], Math.max(rgb[1], rgb[2]));
    float cmin = Math.min(rgb[0], Math.min(rgb[1], rgb[2]));
    float diff = cmax - cmin;
    float h = -1, s;

    if (cmax == cmin) {
      h = 0;
    }

    else if (cmax == rgb[0]) {
      h = (60 * ((rgb[1] - rgb[2]) / diff) + 360) % 360;
    }

    // if cmax equal g then compute h
    else if (cmax == rgb[1]) {
      h = (60 * ((rgb[2] - rgb[0]) / diff) + 120) % 360;
    }

    // if cmax equal b then compute h
    else if (cmax == rgb[2]) {
      h = (60 * ((rgb[0] - rgb[1]) / diff) + 240) % 360;
    }

    // if cmax equal zero
    if (cmax == 0) {
      s = 0;
    } else {
      s = (diff / cmax) * 100;
    }

    h /= 360;
    s /= 100;
    float b = cmax * 255;

    return new float[] { h, s, b };
  }

  public void setAllRed() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGB(i, 255, 0, 0);
    }
  }

  public void setAllBlue() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGB(i, 0, 0, 255);
    }
  }

  public void setAllOff() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGB(i, 0, 0, 0);
    }
  }

  public void setAllAllianceColor() {
    // if(DriverStation.getAlliance() == Alliance.Red) {
    // setAllRed();
    // } else {
    // setAllBlue();
    // }
  }

  public void runBlue() {
    ticksPerSecond = 20;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGB((i + counter) % Constants.LED_LENGTH, 0, 0,
          (int) ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1) * 50);
    }
  }

  public void runRed() {
    ticksPerSecond = 20;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGB((i + counter) % Constants.LED_LENGTH,
          (int) ((-Math.cos((2 * Math.PI * 2 * i) / Constants.LED_LENGTH)) + 1) * 50, 0, 0);
    }
  }

  public void runRainbow() {
    ticksPerSecond = 50;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setHSV((i + counter) % Constants.LED_LENGTH, (int) (((double) i / Constants.LED_LENGTH) * 180), 255,
          255);

    }
  }

  public void runSilly() {
    ticksPerSecond = 5;
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setHSV(i, (int) (Math.random() * 180), 255, 255);
    }
  }

  // run square wave of alliance color
  public void runDisabled() {
    ticksPerSecond = 50;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        runSquareWave(Color.BLUE, -0.4f, 8f);
      } else if (alliance.get() == Alliance.Red) {
        runSquareWave(Color.RED, -0.4f, 8f);
      } else {
        runSquareWave(Color.WHITE, -0.4f, 8f);
      }
    } else {
      runRainbow();
    }
  }

  // if there is a note flash on and off really fast, if there's not a note run
  // disabled pattern
  public void runIntake() {
    ticksPerSecond = 50;
    if (RobotContainer.shooter.hasNote() || RobotContainer.intake.hasNoteInside()) {
      if (counter % 3 == 0) {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
          setRGB(i, 255, 50, 0);
        }
      } else {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
          setRGB(i, 255, 50, 0);
        }
      }
    } else {
      // runSquareWave(new Color(255, 50, 0), -0.6f, 10f);
      for (int i = 0; i < Constants.LED_LENGTH; i++) {
        setRGB(i, 0, 0, 0);
      }
    }
  }

  // if there's a note, just show orange, when the note leaves flash white on
  // and off really fast
  public void runShoot() {
    // var atSetpoint = RobotContainer.shooter.isShooterRampedUp(1);
    // var goodTrajectory = RobotContainer.shooter.goodTrajectory;

    if (RobotContainer.shooter.hasNote()) {
      for (int i = 0; i < Constants.LED_LENGTH; i++) {
        setRGB(i, 255, 50, 0);
      }
    } else {
      if ((counter / 5) % 2 == 0) {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
          setRGB(i, 255, 255, 255);
        }
      } else {
        for (int i = 0; i < Constants.LED_LENGTH; i++) {
          setRGB(i, 0, 0, 0);
        }
      }
    }

  }

  public void runStowed() {
    for (int i = 0; i < Constants.LED_LENGTH; i++) {
      setRGB(i, 255, 50, 0);
    }
  }

  public void runAmp() {
    runShoot();
  }

  public void runSquareWave(Color c, float speed, float sections) {
    float[] hsb = getHSB(c);
    this.speedFactor = speed;
    this.sections = sections;
    currentPosition += speedFactor;

    for (int i = 0; i < pixelArray.length; i++) {
      float adjustedPosition = (i + currentPosition) % pixelArray.length;

      if (adjustedPosition < 0) {
        adjustedPosition += pixelArray.length;
      }

      float j = (float) Math.abs(Math.pow(Math.sin((((Math.PI / 2) * adjustedPosition) * (0.02 * sections))), 2));

      if (j > 0.5) {
        j = hsb[2];
      } else {
        j = 0;
      }

      setPixelColorHSB(i, hsb[0], hsb[1], j);
    }
  }

  public void update() {
    // when disabled show the alliance color
    // when in auto show all the same stuff as in tele probably
    // when in tele and intake it out show something
    // when in tele and intake is out and has a note flash orange
    // when in tele and stowed with note show orange
    // when in tele and stowed without note show... nothing? alliance color?
    // rainbow? idk
    // when in tele and ramping show "graph" of shooter velocity, show green when at
    // setpoint (hopefully it'll shoot immediately)
    // when in tele and shooting do an orange wave effect because it looks cool
    // when climbing show something special I've got lots of patterns made already
    switch (state) {
      case kDisabled:
        runDisabled();
        break;
      case kIntake:
        runIntake();
        break;
      case kShoot:
        runShoot();
        break;
      case kStowed:
        runStowed();
        break;
      case kAmp:
        runAmp();
        break;
    }

    strip.setData(buffer);
    // strip2.setData(buffer);
    counter++;
  }

  private void updateState() {
    var isDisabled = RobotState.isDisabled();
    // var isScoringSpeaker = RobotContainer.shooter.veloctiySetpoint == 15 || RobotContainer.shooter.veloctiySetpoint == 20;
    // var isScoringAmp = Math.abs(RobotContainer.arm.getPosition() - ArmState.kAmp.rotations) < 0.02;
    // var hasIntakeDeployed = RobotContainer.intake.isIntakeDown();

    // if (isScoringSpeaker)
    //   state = LEDState.kShoot;
    // else if (isScoringAmp)
    //   state = LEDState.kAmp;
    // else if (hasIntakeDeployed)
    //   state = LEDState.kIntake;
    // else if (isDisabled)
    //   state = LEDState.kDisabled;
    // else 
    //   state = LEDState.kStowed;
    if (isDisabled) {
      state = LEDState.kDisabled;
    } else {
      state = LEDState.kIntake;
    }

    SmartDashboard.putString("LED State", state.toString());
  }

  public enum LEDState {
    kDisabled,
    kIntake,
    kShoot,
    kStowed,
    kAmp;
  }
}