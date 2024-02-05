package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.Logger;

public class LED {
  public static final int NUM_LEDS = 5;

  private static LED instance;

  public static LED getInstance() {

    if (instance == null) {
      instance = new LED(new LEDIOSim());
    }

    return instance;
  }

  private final LEDIO io;
  private LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();
  private int[] buffer = new int[NUM_LEDS * 3];

  private LED(LEDIO io) {
    this.io = io;
  }

  public void setRGB(int i, int r, int g, int b) {
    buffer[i * 3] += r;
    buffer[i * 3 + 1] += g;
    buffer[i * 3 + 2] += b;
  }

  public void setHSV(int i, int h, int s, int v) {
    buffer[i * 3] += h;
    buffer[i * 3 + 1] += s;
    buffer[i * 3 + 2] += v;
  }

  public void setRangeRGB(int low, int high, int r, int g, int b) {
    for (int i = low; i < high; i++) {
      setRGB(i, r, g, b);
    }
  }

  public void setRangeHSV(int low, int high, int h, int s, int v) {
    for (int i = low; i < high; i++) {
      setHSV(i, h, s, v);
    }
  }

  public void setAllRGB(int r, int g, int b) {
    setRangeRGB(0, NUM_LEDS - 1, r, g, b);
  }

  public void setAllHSV(int h, int s, int v) {
    setRangeHSV(0, NUM_LEDS - 1, h, s, v);
  }

  public void angleToPositionRGB(int angle, int r, int b, int g) {
    buffer[(int) ((angle + 0.5) / 10)] = r;
    buffer[(int) ((angle + 0.5) / 10) + 1] = g;
    buffer[(int) ((angle + 0.5) / 10) + 2] = b;
  }

  public void angleToPositionHSV(int angle, int h, int s, int v) {
    buffer[(int) ((angle + 0.5) / 10)] = h;
    buffer[(int) ((angle + 0.5) / 10) + 1] = s;
    buffer[(int) ((angle + 0.5) / 10) + 2] = v;
  }

  public void angleRangesToPositionRGB(int lowAngle, int highAngle, int r, int b, int g) {
    for (int i = (int) ((lowAngle + 0.5) / 10); i < (int) ((highAngle + 0.5) / 10) * 3; i += 3) {
      buffer[i] = r;
      buffer[i + 1] = g;
      buffer[i + 2] = b;
    }
  }

  public void angleRangesToPositionHSV(int lowAngle, int highAngle, int h, int s, int v) {
    for (int i = (int) ((lowAngle + 0.5) / 10); i < (int) ((highAngle + 0.5) / 10) * 3; i += 3) {
      buffer[i] = h;
      buffer[i + 1] = s;
      buffer[i + 2] = v;
    }
  }

  public void setfixedPosition() {}

  public void clearBuffer() {
    for (int i = 0; i < buffer.length; i++) {
      buffer[i] = 0;
    }
  }

  public void render() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);

    io.setColours(true, buffer);
    clearBuffer();
  }
}
