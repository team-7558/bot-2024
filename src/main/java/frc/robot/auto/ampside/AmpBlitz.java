package frc.robot.auto.ampside;

import frc.robot.G;
import frc.robot.auto.AltAuto;

public class AmpBlitz extends AltAuto {

  public AmpBlitz(int idx) {
    super("AmpBlitz", true);
    trajstack.appendChain().append("AmpSideBlitz456", true);

    trajstack.appendChain().append("AmpSideBlitz546", true);

    trajstack.setActiveIdx(idx);
  }

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    // ss.queueSetpointsLive();
  }

  @Override
  public void onExecute() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    // ss.queueSetpointsLive();
  }
}
