package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.SS;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util;

public abstract class AltAuto {

  private String name;

  public final Trajstack trajstack;

  protected final Drive drive;
  protected final SS ss;

  private Timer t;

  public AltAuto(String s) {
    name = s;
    drive = Drive.getInstance();
    ss = SS.getInstance();
    trajstack = new Trajstack();
    t = new Timer();
  }

  protected abstract void onInit();

  protected abstract void onExecute();

  public final void init() {
    if (!trajstack.isGenerated()) {
      trajstack.generate();
    }

    drive.setCurrentState(drive.PATHING);

    t.reset();
    t.start();
    onInit();
  }

  public final void execute() {
    onExecute();
  }

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
