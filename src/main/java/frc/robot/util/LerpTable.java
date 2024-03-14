package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.Comparator;
import java.util.Scanner;
import org.opencv.core.Point;

public class LerpTable {
  private InterpolatingDoubleTreeMap map;
  private boolean isSorted;
  private String filepath;

  private class PointComparator implements Comparator<Point> {
    @Override
    public int compare(Point p1, Point p2) {
      return (int) Math.signum(p2.x - p1.x); // Descending Order
    }
  }

  public LerpTable() {
    map = new InterpolatingDoubleTreeMap();
    isSorted = false;
  }

  public LerpTable(String filePath) {
    map = new InterpolatingDoubleTreeMap();
    isSorted = false;
    this.filepath = filePath;
    readFromFile();
  }

  public LerpTable compile() {
    isSorted = true;
    return this;
  }

  public LerpTable addPoint(double x, double y) {
    isSorted = false;
    map.put(x, y);
    return this;
  }

  public double calcY(double x) {
    return map.get(x);
  }

  public void readFromFile() {
    String path = Filesystem.getDeployDirectory().toPath().resolve(filepath).toString();
    File file = new File(path);
    if (file.exists()) {
      try {
        Scanner sc = new Scanner(file);
        System.out.println(sc.nextLine());
        while (true) {
          if (sc.hasNext()) {
            double x = sc.nextDouble();
            double y = sc.nextDouble();
            addPoint(x, y);
            // System.out.println(x + " " + y);
          } else {
            break;
          }
        }
        isSorted = true;
        sc.close();
        return;
      } catch (Exception e) {
        e.printStackTrace();
      }
    } else {
      System.out.println(path + " doesn't exist.");
    }
  }
}
