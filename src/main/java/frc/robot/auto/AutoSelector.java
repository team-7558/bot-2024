package frc.robot.auto;

import frc.robot.subsystems.LED.LED;
import java.util.ArrayList;
import java.util.List;

public class AutoSelector {

  private class Struct {
    AltAuto auto;
    int r, g, b;

    public Struct(AltAuto auto, int r, int g, int b) {
      this.auto = auto;
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  List<Struct> autos;

  int size = 0;
  int currIdx = -1, generatedIdx = -1;

  public AutoSelector() {
    autos = new ArrayList<>();
  }

  public AutoSelector add(AltAuto a, int r, int g, int b) {
    Struct s = new Struct(a, r, g, b);
    autos.add(s);
    size++;
    if (currIdx == -1) {
      currIdx = 0;
    }
    return this;
  }

  public void setCurrIdx(int idx) {
    if (size == 0) return;
    currIdx = idx % size;
  }

  public void generate() {
    if (size == 0) {
      System.err.println("No Autos to generate");
    } else {
      generatedIdx = currIdx;
    }
  }

  public void drawLEDS() {
    if (size == 0) return;
    Struct s = autos.get(currIdx);
    LED.getInstance().drawRow(0, s.r, s.g, s.b);
  }
}
