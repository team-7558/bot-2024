package frc.robot.auto.ampside;

import frc.robot.G;
import frc.robot.SS.State;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class SourceSeries extends AltAuto {
  public SourceSeries(int idx) {
    super("SourceSeries", true);
    trajstack
        .appendChain()
        .append("SourceStart", false)
        .append("ForkTo8ToFork", false)
        .append("ForkTo7ToFork", false)
        .append("ForkTo6ToFork", false);

    trajstack
        .appendChain()
        .append("SourceStart", false)
        .append("ForkTo7ToFork", false)
        .append("ForkTo8ToFork", false)
        .append("ForkTo6ToFork", false);

    trajstack.setActiveIdx(idx);
  }

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    ss.autoPreset(new Setpoints(42, 0, -0.02 * i, 0.09));
  }
  
  @Override
  public void onExecute() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    if (after(1.4)) {
      ss.queueState(State.SHOOTING);
    } else if (after(2.2)) {
      ss.queueState(State.IDLE);
    }
  }
}
