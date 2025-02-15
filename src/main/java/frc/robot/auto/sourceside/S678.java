package frc.robot.auto.sourceside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class S678 extends AltAuto {

  public S678() {
    super("S678", true);
    // trajstack.appendChain().append("SourceCore876", false);
    // trajstack.appendChain().append("SourceCore786", false);
    trajstack
        .appendChain()
        .append("SourceStart6", false)
        .append("ForkTo7ToFork", false)
        .append("ForkTo8ToFork", false)
        .append("ForkToSource", false);

    trajstack
        .appendChain()
        .append("SourceStart6", false)
        .append("ForkTo7Bail8ToFork", false)
        .append("ForkTo5ToFork", false)
        .append("ForkToSource", false);

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
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.077), true);
        if (after(segEnd(1) - 1.9) && !ss.hasGamePiece()) {
          trajstack.setActiveIdx(1);
        }
      } else if (before(segEnd(1) + 1.28)) {
        ss.autoShoot();
      } else if (before(segEnd(2) + 0.03)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), true);
      } else if (before(segEnd(2) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(3))) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), true);
      }

    } else if (trajstack.getActiveIdx() == 1) { // Bail 685
      LED.getInstance().setAllRGB(128, 0, 128);
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
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.077), true);
      } else if (before(segEnd(1) + 1.28)) {
        ss.autoShoot();
      } else if (before(segEnd(2) + 0.03)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), true);
      } else if (before(segEnd(2) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(3))) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), true);
      }
    }
  }
}
