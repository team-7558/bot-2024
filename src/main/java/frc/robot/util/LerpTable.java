package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Scanner;
import org.opencv.core.Point;

public class LerpTable {
  private ArrayList<Point> points;
  private boolean isSorted;
  private String filepath;

  private class PointComparator implements Comparator<Point> {
    @Override
    public int compare(Point p1, Point p2) {
      return (int) Math.signum(p2.x - p1.x); // Descending Order
    }
  }

  public LerpTable() {
    points = new ArrayList<Point>();
    isSorted = false;
  }

  public LerpTable(String filePath) {
    points = new ArrayList<Point>();
    isSorted = false;
    this.filepath = filePath;
    readFromFile();
  }

  public LerpTable compile() {
    Collections.sort(points, new PointComparator());
    for (var point : points) {
      System.out.println(point.x + " " + point.y);
    }
    isSorted = true;
    return this;
  }

  public LerpTable addPoint(double x, double y) {
    isSorted = false;
    points.add(new Point(x, y));
    return this;
  }

  public double calcY(double x) {
    if (points.size() >= 2) {
      if (!isSorted) {
        compile();
      }
      if (x >= points.get(0).x) {
        return points.get(0).y;
      }
      for (int i = 0; i < points.size() - 1; i++) {
        if (x <= points.get(i + 1).x && x >= points.get(i + 1).x) {
          // System.out.println(x + ": " + points.get(i+1) + points.get(i) + " :" + Util.remap(
          // points.get(i+1).x,
          // points.get(i).x,
          // x,
          // points.get(i+1).y,
          // points.get(i).y));
          return Util.remap(
              points.get(i + 1).x, points.get(i).x, x, points.get(i + 1).y, points.get(i).y);
        }
      }
      int end = points.size() - 1;

      // System.out.println(x + ": " + points.get(end) + points.get(end-1) + " :" + Util.remap(
      //     points.get(end).x,
      //     points.get(end-1).x,
      //     x,
      //     points.get(end).y,
      //     points.get(end-1).y));
      return Util.remap(
          points.get(end).x, points.get(end - 1).x, x, points.get(end).y, points.get(end - 1).y);
    }
    return 0;
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
