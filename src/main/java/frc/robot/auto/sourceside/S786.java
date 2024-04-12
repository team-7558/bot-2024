package frc.robot.auto.sourceside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class S786 extends AltAuto {

  public S786() {
    super("S786", true);
    // trajstack.appendChain().append("SourceCore876", false);
    // trajstack.appendChain().append("SourceCore786", false);
    trajstack
        .appendChain()
        .append("SourceStart7", false)
        .append("ForkTo8ToFork", false)
        .append("ForkTo6ToFork", false)
        .append("ForkToSource", false);

    trajstack
        .appendChain()
        .append("SourceStart7", false)
        .append("ForkTo8Bail6ToFork", false)
        .append("ForkTo5ToFork", false)
        .append("ForkToSource", false);

    trajstack.setActiveIdx(0);
  }

  Setpoints firstShot = new Setpoints(24, 0, 0.115, G.isRedAlliance() ? 0.091 : 0.091);

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    ss.autoPreset(firstShot, false);
  }

  @Override
  public void onExecute() {
    boolean critical = false;
    double i = G.isRedAlliance() ? 1.0 : -1.0;

    if (trajstack.getActiveIdx() == 0) {

      if (before(0.9)) {
        ss.autoPreset(new Setpoints(24, 0, i * 0.115, G.isRedAlliance() ? 0.091 : 0.091), false);
      } else if (before(1.8)) {
        ss.shoot();
      } else if (before(segEnd(0) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), false);
      } else if (before(segEnd(0) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(1) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.077), false);
        if (after(segEnd(1) - 1.65) && !ss.hasGamePiece()) {
          trajstack.setActiveIdx(1);
        }
      } else if (before(segEnd(1) + 1.28)) {
        ss.autoShoot();
      } else if (before(segEnd(2) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), false);
      } else if (before(segEnd(2) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(3))) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), false);
      }

    } else if (trajstack.getActiveIdx() == 1) { // Bail 765
      LED.getInstance().setAllRGB(128, 0, 128);
      /*if (before(0.9)) {
        ss.autoPreset(new Setpoints(24, 0, i * 0.115, G.isRedAlliance() ? 0.091 : 0.091), false);
      } else if (before(1.8)) {
        ss.shoot();
      } else if (before(segEnd(0) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), false);
      } else if (before(segEnd(0) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(1) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), false);
      } else if (before(segEnd(1) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(2) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), false);
      } else if (before(segEnd(2) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(3))) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), false);
      }*/
    }
  }
}
