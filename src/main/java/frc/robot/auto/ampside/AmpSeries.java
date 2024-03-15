package frc.robot.auto.ampside;

import frc.robot.SS.State;
import frc.robot.auto.AltAuto;

public class AmpSeries extends AltAuto {
  public AmpSeries(int idx) {
    super("AmpSeries", true);
    trajstack
        .appendChain()
        .append("Amp Start", false)
        .append("Amp Fork To 4", false)
        .append("Amp Fork To 5", false)
        .append("Amp Fork To 6 To OP", false);

    trajstack
        .appendChain()
        .append("Amp Start", false)
        .append("Amp Fork To 5", false)
        .append("Amp Fork To 4", false)
        .append("Amp Fork To 6 To OP", false);

    trajstack.appendChain().append("Strafe Run", false);
    trajstack.setActiveIdx(idx);
  }

  @Override
  public void onInit() {
    // double i = G.isRedAlliance() ? 1.0 : -1.0;
    // ss.autoPreset(new Setpoints(42, 0, -0.02 * i, 0.09));
    ss.queueState(State.IDLE);
  }

  @Override
  public void onExecute() {
    // if (after(1.4)) {
    //   ss.queueState(State.SHOOTING);
    // } else if (after(2.2)) {
    //   ss.queueState(State.IDLE);
    // } else if (after(15)) {
    //   ss.queueState(State.IDLE);
    // }
  }
}
