package frc.robot.auto.ampside;

import frc.robot.G;
import frc.robot.auto.AltAuto;

public class AmpSeries extends AltAuto {
  public AmpSeries() {
    super("AmpSeries", true);
    trajstack
        .appendChain()
        .append(0.5)
        .append("Ampside 1", false)
        .append("Ampside 2", false)
        .append("Ampside 3", false);

    trajstack.setActiveIdx(0);
  }

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
  }

  @Override
  public void onExecute() {
  }
}
