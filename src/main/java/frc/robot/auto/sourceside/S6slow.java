package frc.robot.auto.sourceside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class S6slow extends AltAuto {

  public S6slow() {
    super("S6slow", true);
    // trajstack.appendChain().append("SourceCore876", false);
    // trajstack.appendChain().append("SourceCore786", false);
    trajstack.appendChain().append("SourceStart6slow", false).append("ForkTo6ToForkslow", false);

    trajstack.setActiveIdx(0);
  }

  Setpoints RED_SOURCE_AUTO_FIRST_SHOT6 = new Setpoints(34, 0, 0.125, 0.082);
  Setpoints BLUE_SOURCE_AUTO_FIRST_SHOT6 = new Setpoints(34, 0, -0.125, 0.082);

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    ss.autoPreset(
        G.isRedAlliance() ? RED_SOURCE_AUTO_FIRST_SHOT6 : BLUE_SOURCE_AUTO_FIRST_SHOT6, false);
  }

  @Override
  public void onExecute() {
    boolean critical = false;
    double i = G.isRedAlliance() ? 1.0 : -1.0;

    if (trajstack.getActiveIdx() == 0) {

      if (before(0.9)) {
        ss.autoPreset(
            G.isRedAlliance() ? RED_SOURCE_AUTO_FIRST_SHOT6 : BLUE_SOURCE_AUTO_FIRST_SHOT6, false);
      } else if (before(1.6)) {
        ss.firstShoot();
      } else if (before(segEnd(0) + 0.03)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), true);
      } else if (before(segEnd(0) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(1) + 0.03)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), true);
      } else if (before(segEnd(1) + 1.28)) {
        ss.autoShoot();
      } else {
        ss.idle();
      }
    }
  }
}
