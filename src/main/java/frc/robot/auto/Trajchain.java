package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;

public class Trajchain implements IFollowable {

  private List<Traj> trajs;

  private int chainSize, firstPath;

  private Pose2d startingPose;

  private List<Double> startTimes;
  private double totalTime_s;

  private boolean generated = false;

  public Trajchain() {
    trajs = new ArrayList<>();
    startTimes = new ArrayList<>();
    chainSize = 0;
    firstPath = -1;
    totalTime_s = 0;
  }

  public Trajchain append(String filename, boolean isChoreo) {
    Traj t = new Traj(filename, isChoreo);
    trajs.add(t);
    if (firstPath == -1) firstPath = chainSize;
    chainSize++;
    return this;
  }

  public Trajchain append(double delay_s) {
    Traj t = new Traj(delay_s);
    trajs.add(t);
    chainSize++;
    return this;
  }

  @Override
  public void generate() {
    if (firstPath == -1) {
      System.err.println("Path of just waits");
    } else {
      trajs.get(firstPath).generate();
      State initState = trajs.get(firstPath).getInitState();
      startingPose = new Pose2d(initState.positionMeters, initState.targetHolonomicRotation);
      startTimes.add(0.0);
      totalTime_s = 0;
      for (int i = firstPath + 1; i < chainSize; i++) {
        trajs.get(i).generate(trajs.get(i - 1).getEndState());
      }
      for (int i = 0; i < firstPath; i++) {
        trajs.get(i).generate(initState);
      }
      for (int i = 1; i <= chainSize; i++) {
        // System.out.println(i + ": " + trajs.get(i - 1).endTime());
        totalTime_s += trajs.get(i - 1).endTime();
        startTimes.add(totalTime_s);
      }

      for (var time : startTimes) {
        // System.out.println(time);
      }

      System.out.println("Trajchain generated");
      generated = true;
    }
  }

  @Override
  public boolean isGenerated() {
    return generated;
  }

  @Override
  public State sample(double time_s) {
    if (time_s >= startTimes.get(chainSize)) {
      return getEndState();
    } else {

      int i = -1;
      while (time_s >= startTimes.get(i + 1)) i++;
      // System.out.println(i + ": " + (time_s - startTimes.get(i)));

      return trajs.get(i).sample(time_s - startTimes.get(i));
    }
  }

  @Override
  public double endTime() {
    if (generated) {
      return trajs.get(chainSize - 1).endTime();
    }
    return 0.0;
  }

  @Override
  public State getInitState() {
    if (generated) {
      return trajs.get(0).getInitState();
    }
    return null;
  }

  @Override
  public State getEndState() {
    if (generated) {
      return trajs.get(chainSize - 1).getEndState();
    }
    return null;
  }
}
