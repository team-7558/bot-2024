package frc.robot.auto.ampside;

import frc.robot.SS.State;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class DefaultMovingWhileShooting extends AltAuto {

  public DefaultMovingWhileShooting() {
    super("DefaultMovingWhileShooting", true);
    trajstack.appendChain().append("Default Path", false);
    trajstack.setActiveIdx(0);
    trajstack.generate();
  }

  @Override
  public void onInit() {}

  @Override
  public void onExecute() {
    if (before(1)) {
    } else if (between(1, 2)) {
      ss.queueSetpointsLive();
      ss.queueState(State.TRACKING);
    } else if (between(2.0, 2.7)) {
      ss.queueSetpointsLive();
      ss.queueState(State.SHOOTING);
    } else if (between(5.3, 6.3)) {
      // ss.queueSetpoints(new Setpoints(37, 0, 0.02, 0.15));
      ss.queueSetpointsLive();
      ss.queueState(State.TRACKING);
    } else if (between(6.3, 6.9)) {
      ss.queueSetpointsLive();
      ss.queueState(State.SHOOTING);
    } else if (between(9.3, 10.3)) {
      // ss.queueSetpoints(new Setpoints(37, 0, -0.04, 0.155));
      ss.queueSetpointsLive();
      ss.queueState(State.TRACKING);
    } else if (between(10.3, 10.9)) {
      ss.queueSetpointsLive();
      ss.queueState(State.SHOOTING);
    } else if (between(12.3, 13.3)) {
      // ss.queueSetpoints(new Setpoints(45, 0, -0.06, 0.125));
      ss.queueSetpointsLive();
      ss.queueState(State.TRACKING);
    } else if (between(13.3, 13.9)) {
      ss.queueSetpointsLive();
      ss.queueState(State.SHOOTING);
    } else {
      ss.queueSetpoints(new Setpoints(40, 8, 0, 0.05));
      ss.chamber();
    }
  }
}
