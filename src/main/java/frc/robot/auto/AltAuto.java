package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SS;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util;
import java.util.ArrayList;
import java.util.List;

public abstract class AltAuto extends Command {

  private Command autoPath;
  private String name;
  private SequentialCommandGroup pathSegments;
  private List<Double> segmentTimes;
  private double totalTime;
  private int segments;
  private boolean generated;

  protected final Drive drive;
  protected final SS ss;

  private Timer t;

  public AltAuto(String name) {
    this.name = name;

    pathSegments = new SequentialCommandGroup();
    segmentTimes = new ArrayList<>();
    totalTime = 0;
    drive = Drive.getInstance();
    ss = SS.getInstance();
    t = new Timer();
  }

  public void generate() {
    if (!generated) {
      autoPath = new ParallelCommandGroup(pathSegments);
      generated = true;
      if (segments == 1) System.out.println("Auto generated: " + name + " with 1 segment");
      else System.out.println("Auto generated: " + name + " with " + segments + " segments");
    }
  }

  public Command getCommand() {
    if (!generated) generate();
    return autoPath;
  }

  public AltAuto addPath(String path, boolean isChoreo) {
    PathPlannerPath pppath =
        isChoreo ? PathPlannerPath.fromChoreoTrajectory(path) : PathPlannerPath.fromPathFile(path);
    totalTime +=
        pppath
            .getTrajectory(new ChassisSpeeds(), drive.getRotation())
            .getTotalTimeSeconds(); // Fill in starting speed and rotation
    Command apath = AutoBuilder.pathfindThenFollowPath(pppath, new PathConstraints(3, 3, 20, 10));
    Command bpath = AutoBuilder.followPath(pppath);
    pathSegments.addCommands(bpath);

    segmentTimes.add(totalTime);
    segments++;
    return this;
  }

  public AltAuto addWait(double delay_s) {
    totalTime += delay_s;
    pathSegments.addCommands(new WaitCommand(delay_s));
    segmentTimes.add(totalTime);
    segments++;
    return this;
  }

  public void initialize() {
    t.restart();
  }

  @Override
  public abstract void execute();

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interuppted) {}

  protected boolean before(double time_s) {
    return t.get() < time_s;
  }

  protected boolean after(double time_s) {
    return t.get() > time_s;
  }

  protected boolean between(double time0_s, double time1_s) {
    return after(time0_s) && before(time1_s);
  }

  protected double alpha(double time0_s, double time1_s) {
    return Util.unlerp(time0_s, time1_s, t.get());
  }
}
