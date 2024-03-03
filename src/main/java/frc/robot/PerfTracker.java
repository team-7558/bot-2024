package frc.robot;

import frc.robot.util.Util;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PerfTracker {

  private static List<Long> perfmap;
  private static List<String> nameList;
  private static int size;

  private static long prev = -1;

  static {
    perfmap = new ArrayList<>(16);
    nameList = new ArrayList<>(16);
    size = 0;
  }

  public static int start(String name) {
    int id = nameList.indexOf(name);
    if (id < 0) {
      id = size;
      nameList.add(name);
      size++;
      perfmap.add(Logger.getRealTimestamp());
    } else {
      perfmap.set(id, Logger.getRealTimestamp());
    }
    return id;
  }

  public static void end(int id) {
    perfmap.set(id, Logger.getRealTimestamp() - perfmap.get(id));
  }

  public static void periodic() {
    long sum = 0;
    for (int i = 0; i < size; i++) {
      long delta = perfmap.get(i);
      Logger.recordOutput("PerfMs/" + nameList.get(i), Util.FPGATimeDelta_ms(delta));
      sum += delta;
    }
    Logger.recordOutput("PerfMs/Sum", Util.FPGATimeDelta_ms(sum));
    if (prev < 0) {
      prev = Logger.getRealTimestamp();
    } else {
      long curr = Logger.getRealTimestamp();
      Logger.recordOutput("PerfMs/RobotPeriodic", Util.FPGATimeDelta_ms(curr, prev));
      prev = curr;
    }
  }
}
