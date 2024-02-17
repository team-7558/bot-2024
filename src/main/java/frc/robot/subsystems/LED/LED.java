package frc.robot.subsystems.LED;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class LED {
  public static final int WIDTH = 32;
  public static final int HEIGHT = 8;
  public static final int NUM_LEDS = WIDTH * HEIGHT;

  private static LED instance;

  public static LED getInstance() {

    if (instance == null) {
      instance = new LED(new LEDIOReal());
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

  public void scaleRGB(int i, int r, int g, int b) {
    buffer[i * 3] *= r;
    buffer[i * 3 + 1] *= g;
    buffer[i * 3 + 2] *= b;
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

  public void scaleRangeRGB(int low, int high, int r, int g, int b) {
    for (int i = low; i < high; i++) {
      scaleRGB(i, r, g, b);
    }
  }

  public void setRangeHSV(int low, int high, int h, int s, int v) {
    for (int i = low; i < high; i++) {
      setHSV(i, h, s, v);
    }
  }

  public void setAllRGB(int r, int g, int b) {
    setRangeRGB(0, NUM_LEDS, r, g, b);
  }

  public void setAllHSV(int h, int s, int v) {
    setRangeHSV(0, NUM_LEDS, h, s, v);
  }

  public void drawPoint(int x, int y, int r, int g, int b) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) {
      int i = (x * HEIGHT) + (x % 2 == 1 ? HEIGHT - 1 - y : y);
      setRGB(i, r, g, b);
    }
  }

  public void drawRow(int row, int r, int g, int b) {
    for (int i = 0; i < WIDTH; i++) {
      drawPoint(i, row, r, g, b);
    }
  }

  public void drawCol(int col, int r, int g, int b) {
    for (int i = 0; i < HEIGHT; i++) {
      drawPoint(col, i, r, g, b);
    }
  }

  public void drawRect(int x0, int y0, int x1, int y1, int r, int g, int b) {
    int minX = Math.min(x0, x1);
    int maxX = Math.max(x0, x1);
    int minY = Math.min(y0, y1);
    int maxY = Math.min(y0, y1);

    for (int x = minX; x < maxX; x++) {
      for (int y = minY; y < maxY; y++) {
        drawPoint(x, y, r, g, b);
      }
    }
  }

  public void drawCircle(int x, int y, int rad, int r, int g, int b) {
    for (int t = 0; t < 36; t++) {
      double tr = Units.degreesToRadians(t * 10);
      drawPoint(x + (int) (rad * Math.cos(tr)), y + (int) (rad * Math.sin(tr)), r, g, b);
    }
  }

  public void draw0(int i, int r, int g, int b) {
    drawPoint(i + 1, 2, r, g, b);
    drawPoint(i + 1, 3, r, g, b);
    drawPoint(i + 1, 4, r, g, b);
    drawPoint(i + 1, 5, r, g, b);
    drawPoint(i + 1, 6, r, g, b);

    drawPoint(i + 2, 1, r, g, b);
    drawPoint(i + 2, 6, r, g, b);
    drawPoint(i + 2, 7, r, g, b);

    drawPoint(i + 3, 1, r, g, b);
    drawPoint(i + 3, 5, r, g, b);
    drawPoint(i + 3, 7, r, g, b);

    drawPoint(i + 4, 1, r, g, b);
    drawPoint(i + 4, 4, r, g, b);
    drawPoint(i + 4, 7, r, g, b);

    drawPoint(i + 5, 1, r, g, b);
    drawPoint(i + 5, 3, r, g, b);
    drawPoint(i + 5, 7, r, g, b);

    drawPoint(i + 6, 1, r, g, b);
    drawPoint(i + 6, 2, r, g, b);
    drawPoint(i + 6, 7, r, g, b);

    drawPoint(i + 7, 2, r, g, b);
    drawPoint(i + 7, 3, r, g, b);
    drawPoint(i + 7, 4, r, g, b);
    drawPoint(i + 7, 5, r, g, b);
    drawPoint(i + 7, 6, r, g, b);
  }

  public void draw1(int i, int r, int g, int b) {
    drawPoint(i + 6, 1, r, g, b);

    drawPoint(i + 7, 1, r, g, b);
    drawPoint(i + 7, 2, r, g, b);
    drawPoint(i + 7, 3, r, g, b);
    drawPoint(i + 7, 4, r, g, b);
    drawPoint(i + 7, 5, r, g, b);
    drawPoint(i + 7, 6, r, g, b);
  }

  public void draw2(int i, int r, int g, int b) {
    drawPoint(i + 1, 5, r, g, b);
    drawPoint(i + 1, 6, r, g, b);

    drawPoint(i + 2, 1, r, g, b);
    drawPoint(i + 2, 4, r, g, b);
    drawPoint(i + 2, 7, r, g, b);

    drawPoint(i + 3, 1, r, g, b);
    drawPoint(i + 3, 4, r, g, b);
    drawPoint(i + 3, 7, r, g, b);

    drawPoint(i + 4, 1, r, g, b);
    drawPoint(i + 4, 4, r, g, b);
    drawPoint(i + 4, 7, r, g, b);

    drawPoint(i + 5, 1, r, g, b);
    drawPoint(i + 5, 4, r, g, b);
    drawPoint(i + 5, 7, r, g, b);

    drawPoint(i + 6, 1, r, g, b);
    drawPoint(i + 6, 4, r, g, b);
    drawPoint(i + 6, 7, r, g, b);

    drawPoint(i + 7, 2, r, g, b);
    drawPoint(i + 7, 3, r, g, b);
  }

  public void draw3(int i, int r, int g, int b) {
    drawPoint(i + 2, 1, r, g, b);
    drawPoint(i + 2, 4, r, g, b);
    drawPoint(i + 2, 7, r, g, b);

    drawPoint(i + 3, 1, r, g, b);
    drawPoint(i + 3, 4, r, g, b);
    drawPoint(i + 3, 7, r, g, b);

    drawPoint(i + 4, 1, r, g, b);
    drawPoint(i + 4, 4, r, g, b);
    drawPoint(i + 4, 7, r, g, b);

    drawPoint(i + 5, 1, r, g, b);
    drawPoint(i + 5, 4, r, g, b);
    drawPoint(i + 5, 7, r, g, b);

    drawPoint(i + 6, 1, r, g, b);
    drawPoint(i + 6, 4, r, g, b);
    drawPoint(i + 6, 7, r, g, b);

    drawPoint(i + 7, 2, r, g, b);
    drawPoint(i + 7, 3, r, g, b);
    drawPoint(i + 7, 5, r, g, b);
    drawPoint(i + 7, 6, r, g, b);
  }

  public void draw4(int i, int r, int g, int b) {
    drawPoint(i + 1, 1, r, g, b);
    drawPoint(i + 1, 2, r, g, b);
    drawPoint(i + 1, 3, r, g, b);

    drawPoint(i + 2, 4, r, g, b);

    drawPoint(i + 3, 4, r, g, b);

    drawPoint(i + 4, 4, r, g, b);

    drawPoint(i + 5, 4, r, g, b);

    drawPoint(i + 6, 4, r, g, b);

    drawPoint(i + 7, 1, r, g, b);
    drawPoint(i + 7, 2, r, g, b);
    drawPoint(i + 7, 3, r, g, b);
    drawPoint(i + 7, 4, r, g, b);
    drawPoint(i + 7, 5, r, g, b);
    drawPoint(i + 7, 6, r, g, b);
  }

  public void draw5(int i, int r, int g, int b) {
    drawPoint(i + 1, 2, r, g, b);
    drawPoint(i + 1, 3, r, g, b);
    drawPoint(i + 1, 4, r, g, b);

    drawPoint(i + 2, 1, r, g, b);
    drawPoint(i + 2, 4, r, g, b);
    drawPoint(i + 2, 7, r, g, b);

    drawPoint(i + 3, 1, r, g, b);
    drawPoint(i + 3, 4, r, g, b);
    drawPoint(i + 3, 7, r, g, b);

    drawPoint(i + 4, 1, r, g, b);
    drawPoint(i + 4, 4, r, g, b);
    drawPoint(i + 4, 7, r, g, b);

    drawPoint(i + 5, 1, r, g, b);
    drawPoint(i + 5, 4, r, g, b);
    drawPoint(i + 5, 7, r, g, b);

    drawPoint(i + 6, 1, r, g, b);
    drawPoint(i + 6, 4, r, g, b);
    drawPoint(i + 6, 7, r, g, b);

    drawPoint(i + 7, 5, r, g, b);
    drawPoint(i + 7, 6, r, g, b);
  }

  public void draw6(int i, int r, int g, int b) {
    drawPoint(i + 1, 2, r, g, b);
    drawPoint(i + 1, 3, r, g, b);
    drawPoint(i + 1, 4, r, g, b);
    drawPoint(i + 1, 5, r, g, b);
    drawPoint(i + 1, 6, r, g, b);

    drawPoint(i + 2, 1, r, g, b);
    drawPoint(i + 2, 4, r, g, b);
    drawPoint(i + 2, 7, r, g, b);

    drawPoint(i + 3, 1, r, g, b);
    drawPoint(i + 3, 4, r, g, b);
    drawPoint(i + 3, 7, r, g, b);

    drawPoint(i + 4, 1, r, g, b);
    drawPoint(i + 4, 4, r, g, b);
    drawPoint(i + 4, 7, r, g, b);

    drawPoint(i + 5, 1, r, g, b);
    drawPoint(i + 5, 4, r, g, b);
    drawPoint(i + 5, 7, r, g, b);

    drawPoint(i + 6, 1, r, g, b);
    drawPoint(i + 6, 4, r, g, b);
    drawPoint(i + 6, 7, r, g, b);

    drawPoint(i + 7, 5, r, g, b);
    drawPoint(i + 7, 6, r, g, b);
  }

  public void draw7(int i, int r, int g, int b) {
    drawPoint(i + 1, 1, r, g, b);

    drawPoint(i + 2, 1, r, g, b);
    drawPoint(i + 2, 6, r, g, b);
    drawPoint(i + 2, 7, r, g, b);

    drawPoint(i + 3, 1, r, g, b);
    drawPoint(i + 3, 5, r, g, b);
    drawPoint(i + 3, 6, r, g, b);

    drawPoint(i + 4, 1, r, g, b);
    drawPoint(i + 4, 4, r, g, b);
    drawPoint(i + 4, 5, r, g, b);

    drawPoint(i + 5, 1, r, g, b);
    drawPoint(i + 5, 3, r, g, b);
    drawPoint(i + 5, 4, r, g, b);

    drawPoint(i + 6, 1, r, g, b);
    drawPoint(i + 6, 2, r, g, b);
    drawPoint(i + 6, 3, r, g, b);

    drawPoint(i + 7, 1, r, g, b);
    drawPoint(i + 7, 2, r, g, b);
  }

  public void draw8(int i, int r, int g, int b) {
    drawPoint(i + 1, 2, r, g, b);
    drawPoint(i + 1, 3, r, g, b);
    drawPoint(i + 1, 5, r, g, b);
    drawPoint(i + 1, 6, r, g, b);

    drawPoint(i + 2, 1, r, g, b);
    drawPoint(i + 2, 4, r, g, b);
    drawPoint(i + 2, 7, r, g, b);

    drawPoint(i + 3, 1, r, g, b);
    drawPoint(i + 3, 4, r, g, b);
    drawPoint(i + 3, 7, r, g, b);

    drawPoint(i + 4, 1, r, g, b);
    drawPoint(i + 4, 4, r, g, b);
    drawPoint(i + 4, 7, r, g, b);

    drawPoint(i + 5, 1, r, g, b);
    drawPoint(i + 5, 4, r, g, b);
    drawPoint(i + 5, 7, r, g, b);

    drawPoint(i + 6, 1, r, g, b);
    drawPoint(i + 6, 4, r, g, b);
    drawPoint(i + 6, 7, r, g, b);

    drawPoint(i + 7, 2, r, g, b);
    drawPoint(i + 7, 3, r, g, b);
    drawPoint(i + 7, 5, r, g, b);
    drawPoint(i + 7, 6, r, g, b);
  }

  public void draw9(int i, int r, int g, int b) {
    drawPoint(i + 1, 2, r, g, b);
    drawPoint(i + 1, 3, r, g, b);

    drawPoint(i + 2, 1, r, g, b);
    drawPoint(i + 2, 4, r, g, b);
    drawPoint(i + 2, 7, r, g, b);

    drawPoint(i + 3, 1, r, g, b);
    drawPoint(i + 3, 4, r, g, b);
    drawPoint(i + 3, 7, r, g, b);

    drawPoint(i + 4, 1, r, g, b);
    drawPoint(i + 4, 4, r, g, b);
    drawPoint(i + 4, 7, r, g, b);

    drawPoint(i + 5, 1, r, g, b);
    drawPoint(i + 5, 4, r, g, b);
    drawPoint(i + 5, 7, r, g, b);

    drawPoint(i + 6, 1, r, g, b);
    drawPoint(i + 6, 4, r, g, b);
    drawPoint(i + 6, 7, r, g, b);

    drawPoint(i + 7, 2, r, g, b);
    drawPoint(i + 7, 3, r, g, b);
    drawPoint(i + 7, 4, r, g, b);
    drawPoint(i + 7, 5, r, g, b);
    drawPoint(i + 7, 6, r, g, b);
  }

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
