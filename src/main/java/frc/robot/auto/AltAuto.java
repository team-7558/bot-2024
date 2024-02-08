package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SS;
import frc.robot.subsystems.drive.Drive;
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

  public AltAuto(String name) {
    this.name = name;

    pathSegments = new SequentialCommandGroup();
    segmentTimes = new ArrayList<>();
    totalTime = 0;
    drive = Drive.getInstance();
    ss = SS.getInstance();
  }

  public void generate() {
    autoPath = Commands.parallel(pathSegments, this);
    System.out.println("Auto generated: " + name + " with " + segments + " segments");
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
    Command apath = AutoBuilder.followPath(pppath);
    pathSegments.addCommands(apath);
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

  @Override
  public abstract void initialize();

  @Override
  public abstract void execute();

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interuppted) {}
}
