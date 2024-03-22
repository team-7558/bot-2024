package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.G;
import java.util.Optional;

public class Traj implements IFollowable {

  private enum TrajType {
    MOVING,
    STAYING
  }

  final TrajType type;

  private String filename;
  private boolean isChoreo;
  private double delay_s;

  private PathPlannerTrajectory traj;
  private PathPlannerPath path;
  private State initState;

  private boolean generated = false;

  public Traj(String filename, boolean isChoreo) {
    type = TrajType.MOVING;

    this.filename = filename;
    this.isChoreo = isChoreo;
  }

  public Traj(double delay_s) {
    type = TrajType.STAYING;

    this.delay_s = delay_s;
  }

  @Override
  public void generate() {
    if (this.type == TrajType.MOVING) {
      PathPlannerPath pppath =
          isChoreo
              ? PathPlannerPath.fromChoreoTrajectory(filename)
              : PathPlannerPath.fromPathFile(filename);
      ChassisSpeeds cs = new ChassisSpeeds();
      this.path = pppath;
      double startrad = 0;
      if (G.isRedAlliance()) {
        pppath = pppath.flipPath();
        startrad = Math.PI;
      }

      this.traj = pppath.getTrajectory(cs, Rotation2d.fromRadians(startrad));
      initState = traj.getInitialState();
    } else {
      this.initState = new State();
      this.initState.timeSeconds = delay_s;
      this.initState.positionMeters = new Translation2d(0, 0);
      this.initState.targetHolonomicRotation = Rotation2d.fromRadians(0);
      this.initState.velocityMps = 0;
      this.initState.accelerationMpsSq = 0;
      this.initState.headingAngularVelocityRps = 0;
      this.initState.holonomicAngularVelocityRps = Optional.of(0.0);
      this.initState.curvatureRadPerMeter = -7558.0;
      System.err.println("Why am I here");
    }
    this.generated = true;
  }

  public void generate(State prevState) {
    if (this.type == TrajType.MOVING) {
      PathPlannerPath pppath =
          isChoreo
              ? PathPlannerPath.fromChoreoTrajectory(filename)
              : PathPlannerPath.fromPathFile(filename);
      double rotvel =
          prevState.holonomicAngularVelocityRps.orElse(prevState.headingAngularVelocityRps);
      ChassisSpeeds cs =
          new ChassisSpeeds(
              prevState.velocityMps * Math.cos(Math.PI * 0.25),
              prevState.velocityMps * Math.sin(Math.PI * 0.25),
              rotvel);

      if (G.isRedAlliance()) {
        pppath = pppath.flipPath();
      }

      this.traj = pppath.getTrajectory(cs, prevState.targetHolonomicRotation);
      initState = traj.getInitialState();
    } else {
      this.initState = prevState;
      this.initState.timeSeconds = delay_s;
      this.initState.velocityMps = 0;
      this.initState.accelerationMpsSq = 0;
      this.initState.headingAngularVelocityRps = 0;
      this.initState.holonomicAngularVelocityRps = Optional.of(0.0);
      this.initState.curvatureRadPerMeter = -7558.0;
    }

    this.generated = true;
  }

  @Override
  public boolean isGenerated() {
    return generated;
  }

  @Override
  public double endTime() {
    if (type == TrajType.MOVING) {
      return traj.getTotalTimeSeconds();
    } else {
      return initState.timeSeconds;
    }
  }

  public PathPlannerPath getPath() {
    return this.path;
  }

  @Override
  public State sample(double time) {
    if (type == TrajType.MOVING) {
      return traj.sample(time);
    } else {
      return initState;
    }
  }

  @Override
  public State getInitState() {
    return initState;
  }

  @Override
  public State getEndState() {
    if (type == TrajType.MOVING) {
      return traj.getEndState();
    } else {
      return initState;
    }
  }
}
