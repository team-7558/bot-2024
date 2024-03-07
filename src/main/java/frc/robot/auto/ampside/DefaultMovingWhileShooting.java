package frc.robot.auto.ampside;

import frc.robot.SS.State;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class DefaultMovingWhileShooting extends AltAuto {

  public DefaultMovingWhileShooting() {
    super("DefaultMovingWhileShooting", true);
    trajstack.appendChain()
    .append("Default Path", false);
    trajstack.setActiveIdx(0);
    trajstack.generate();
  }

  @Override
  public void onInit() {}

  @Override
  public void onExecute() {
    if (before(2.0)) {
      ss.trackPreset(new Setpoints(37, 0, 0.03, 0.15), false);
    } else if (between(2.0, 2.7)) {
      ss.queueState(State.SHOOTING);
    } else if (between(2.7, 6.3)) {
      // ss.queueSetpoints(new Setpoints(37, 0, 0.02, 0.15));
      ss.trackPreset(new Setpoints(37, 0, 0.02, 0.15), false);
    } else if (between(6.3, 6.9)) {
      ss.queueState(State.SHOOTING);
    } else if (between(6.9, 10.3)) {
      // ss.queueSetpoints(new Setpoints(37, 0, -0.04, 0.155));
      ss.trackPreset(new Setpoints(37, 0, -0.04, 0.155), false);
    } else if (between(10.3, 10.9)) {
      ss.queueState(State.SHOOTING);
    } else if (between(10.9, 13.3)) {
      // ss.queueSetpoints(new Setpoints(45, 0, -0.06, 0.125));
      ss.trackPreset(new Setpoints(40, 0, -0.06, 0.125), false);
    } else if (between(13.3, 13.9)) {
      ss.queueState(State.SHOOTING);
    }
  }
}
