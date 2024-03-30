package frc.robot.auto.ampside;

import frc.robot.G;
import frc.robot.auto.AltAuto;

public class AmpSeries extends AltAuto {
  public AmpSeries() {
    super("AmpSeries", true);
    trajstack
        .appendChain()
        .append("Ampside Start", false)
        .append("AmpForkTo5ToStrafe", false)
        .append("Amp Strafe Run", false);

    trajstack.setActiveIdx(0);
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
