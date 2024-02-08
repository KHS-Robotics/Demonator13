package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.awt.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.leds.LEDDisabled;

public class NewLEDStrip extends SubsystemBase {
  AddressableLED strip;
  AddressableLEDBuffer buffer;
  public int counter = 0;
  public Color[] pixelArray;

  public NewLEDStrip() {
    pixelArray = new Color[Constants.LED_LENGTH];
    for (Color c : pixelArray) {
      c = new Color(0);
    }

    strip = new AddressableLED(RobotMap.LED_PORT);
    strip.setLength(Constants.LED_LENGTH * 2);
    buffer = new AddressableLEDBuffer(Constants.LED_LENGTH * 2);
    strip.setData(buffer);
    strip.start();

    this.setDefaultCommand(new LEDDisabled());
  }

  public void setColorArray(Color[] colors) {
    for (int i = 0; i < colors.length; i++) {
      buffer.setRGB(i, colors[i].getRed(), colors[i].getGreen(), colors[i].getBlue());
    }
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


    return new float[] {h, s, b};
}

  @Override
  public void periodic() {
    strip.setData(buffer);
    counter++;
  }

}
