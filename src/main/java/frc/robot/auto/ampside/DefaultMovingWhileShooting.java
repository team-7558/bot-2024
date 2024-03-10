package frc.robot.auto.ampside;

import frc.robot.G;
import frc.robot.SS.State;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class DefaultMovingWhileShooting extends AltAuto {

  public DefaultMovingWhileShooting() {
    super("DefaultMovingWhileShooting", true);
    trajstack.appendChain().append("Default Path", false);
    trajstack.setActiveIdx(0);
    // trajstack.generate();
  }

  @Override
  public void onInit() {}

  @Override
  public void onExecute() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    if (before(2.0)) {
      ss.autoPreset(new Setpoints(30, 0, 0.07 * i, 0.175));
    } else if (before(2.7)) {
      ss.queueState(State.SHOOTING);
    } else if (before(6.7)) {
      // ss.queueSetpoints(new Setpoints(37, 0, 0.02, 0.15));
      ss.autoPreset(new Setpoints(33, 0, 0.02 * i, 0.16 * 1.15));
    } else if (before(7.5)) {
      ss.queueState(State.SHOOTING);
    } else if (before(10.3)) {
      // ss.queueSetpoints(new Setpoints(37, 0, -0.04, 0.155));
      ss.autoPreset(new Setpoints(33, 0, -0.02 * i, 0.167));
    } else if (before(10.95)) {
      ss.queueState(State.SHOOTING);
    } else if (before(14.2)) {
      // ss.queueSetpoints(new Setpoints(45, 0, -0.06, 0.125));
      ss.autoPreset(new Setpoints(39, 0, -0.02 * i, 0.13));
    } else if (before(15.0)) {
      ss.queueState(State.SHOOTING);
    } else {
      ss.queueState(State.IDLE);
    }
  }
}
