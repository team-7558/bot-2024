package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.MathUtil;
import frc.robot.G;
import java.util.ArrayList;
import java.util.List;

public class Trajstack implements IFollowable {

  List<IFollowable> trajs;

  private int stackSize, activeTraj;

  private boolean generated = false;

  public Trajstack() {
    trajs = new ArrayList<>();
    stackSize = 0;
    activeTraj = -1;
  }

  public Trajstack append(IFollowable t) {
    trajs.add(t);
    if (activeTraj == -1) activeTraj = stackSize;
    stackSize++;
    return this;
  }

  public Trajchain appendChain() {
    Trajchain tc = new Trajchain();
    trajs.add(tc);
    if (activeTraj == -1) activeTraj = stackSize;
    stackSize++;
    return tc;
  }

  @Override
  public void generate() {
    System.out.println("Generating for " + (G.isRedAlliance() ? "Red" : "Blue"));
    if (activeTraj == -1) {
      System.err.println("No active paths");
    } else {
      for (int i = 0; i < stackSize; i++) {
        trajs.get(i).generate();
      }
      System.out.println("TrajStack generated");
      generated = true;
    }
  }

  @Override
  public boolean isGenerated() {
    return generated;
  }

  @Override
  public State sample(double time_s) {
    return trajs.get(activeTraj).sample(time_s);
  }

  @Override
  public double endTime() {
    if (generated) {
      return trajs.get(activeTraj).endTime();
    }
    return 0.0;
  }

  public int getActiveIdx() {
    return activeTraj;
  }

  public int setActiveIdx(int idx) {
    activeTraj = MathUtil.clamp(idx, 0, stackSize - 1);
    return activeTraj;
  }

  @Override
  public State getInitState() {
    if (generated) {
      return trajs.get(activeTraj).getInitState();
    }
    return null;
  }

  @Override
  public State getEndState() {
    if (generated) {
      return trajs.get(activeTraj).getEndState();
    }
    return null;
  }
}
