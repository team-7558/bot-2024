package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

public interface IFollowable {

  public void generate();

  public boolean isGenerated();

  public State sample(double time_s);

  public double endTime();

  public State getInitState();

  public State getEndState();
}
