package frc.robot.auto.sourceside;

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
    ss.autoPreset(new Setpoints(28, 0, 0.12 * i, 0.05));
  }

  @Override
  public void onExecute() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;

    if(trajstack.getActiveIdx() == 0){
      if (after(1.4)) {
        ss.queueState(State.SHOOTING);
      } else if (after(2.2)) {
        ss.autoPreset(new Setpoints(28, 0, 0.12 * i, 0.05));
      } else if (after(6.0)) {
        ss.queueState(State.SHOOTING);
      } else if (after(6.5)) {
        ss.autoPreset(new Setpoints(28, 0, 0.12 * i, 0.05));
      } else if (after(11.0)) {
        ss.queueState(State.SHOOTING);
      }else if (after(15)) {
        ss.queueState(State.IDLE);
      }
    } else {
      if (after(1.4)) {
        ss.queueState(State.SHOOTING);
      } else if (after(2.2)) {
        ss.autoPreset(new Setpoints(28, 0, 0.12 * i, 0.05));
      } else if (after(6.0)) {
        ss.queueState(State.SHOOTING);
      } else if (after(6.5)) {
        ss.autoPreset(new Setpoints(28, 0, 0.12 * i, 0.05));
      } else if (after(11.0)) {
        ss.queueState(State.SHOOTING);
      }else if (after(15)) {
        ss.queueState(State.IDLE);
      }
    }     

    
  }
}
