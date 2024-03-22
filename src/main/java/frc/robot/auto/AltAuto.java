package frc.robot.auto;

import com.pathplanner.lib.path.EventMarker;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.SS;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util;

public abstract class AltAuto {

  private String name;

  public final Trajstack trajstack;

  protected final Drive drive;
  protected final SS ss;
  protected final LED led;

  private Timer t;
  private boolean forcePoseReset;

  private boolean generated = false;

  public AltAuto(String s, boolean forcePoseReset) {
    name = s;
    drive = Drive.getInstance();
    ss = SS.getInstance();
    led = LED.getInstance();
    trajstack = new Trajstack();
    this.forcePoseReset = forcePoseReset;
    t = new Timer();
  }

  protected abstract void onInit();

  protected abstract void onExecute();

  public final void init() {
    System.out.println("Starting " + name);
    if (!generated) {
      generate();
    }

    if (forcePoseReset) {
      drive.hardSetPose(new Pose2d(trajstack.getInitState().positionMeters, drive.getRotation()));
    }

    drive.setCurrentState(drive.PATHING);

    t.reset();
    t.start();
    onInit();
  }

  @Override
  public String toString() {
    return name;
  }

  public void generate() {
    trajstack.generate();
    generated = true;
  }

  public final void execute() {
    onExecute();
    double time = t.get();
    if (time < 15.0) led.drawPreciseNumber(time, 16, 16, 16);
    else led.drawNumber(time, 48, 0, 0);
  }

  protected boolean before(double time_s) {
    return t.get() < time_s;
  }

  protected boolean after(double time_s) {
    return t.get() > time_s;
  }

  /**
   * Only to be used for single-file choreo trajectories
   *
   * @return
   */
  protected int getLastEventMarkerIndex() {
    int eventMarkerIndex = 0;
    for (IFollowable followable : trajstack.trajs) {

      Traj traj = (Traj) followable;
      for (EventMarker m : traj.getPath().getEventMarkers()) {
        if (m.getWaypointRelativePos() <= t.get()) {
          eventMarkerIndex++;
        }
      }
    }
    return eventMarkerIndex;
  }

  protected boolean between(double time0_s, double time1_s) {
    return after(time0_s) && before(time1_s);
  }

  protected double alpha(double time0_s, double time1_s) {
    return Util.unlerp(time0_s, time1_s, t.get());
  }

  protected boolean near(double x, double y, double tol) {
    double dx = x - drive.getPose().getX();
    double dy = y - drive.getPose().getY();
    return dx * dx + dy * dy < tol * tol;
  }
}
