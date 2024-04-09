package frc.robot.auto.sourceside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class SourceSeries extends AltAuto {

  public SourceSeries(int idx) {
    super("SourceSeries", true);
    // trajstack.appendChain().append("SourceCore876", false);
    // trajstack.appendChain().append("SourceCore786", false);
    trajstack
        .appendChain()
        .append("SourceStart8", false)
        .append("ForkTo7ToFork", false)
        .append("ForkTo6ToFork", false)
        .append("ForkToSource", false);

    trajstack
        .appendChain()
        .append("SourceStart7", false)
        .append("ForkTo8ToFork", false)
        .append("ForkTo6ToFork", false)
        .append("ForkToSource", false);

    trajstack
        .appendChain()
        .append("SourceStart7", false)
        .append("ForkTo6ToFork", false)
        .append("ForkTo8ToFork", false)
        .append("ForkToSource", false);

    trajstack
        .appendChain()
        .append("SourceStart8", false)
        .append("ForkTo6ToFork", false)
        .append("ForkTo7ToFork", false)
        .append("ForkToSource", false);

    trajstack
        .appendChain()
        .append("SourceStart6", false)
        .append("ForkTo7ToFork", false)
        .append("ForkTo8ToFork", false)
        .append("ForkToSource", false);

    trajstack.setActiveIdx(idx);
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
      } else if (before(segEnd(1) + 1.28)) {
        ss.autoShoot();
      } else if (before(segEnd(2) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), false);
      } else if (before(segEnd(2) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(3))) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.03, 0.078), false);
      }

    } else if (trajstack.getActiveIdx() == 1) { // Variant
      if (before(0.9)) {
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
      }
    } else if (trajstack.getActiveIdx() == 2) { // 768
      if (before(0.9)) {
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
      }
    } else if (trajstack.getActiveIdx() == 3) { // 786
      if (before(0.9)) {
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
      }
    } else if (trajstack.getActiveIdx() == 4) { // 687
      if (before(0.9)) {
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
      }
    }

    // intake 3.2
    // shoot 5.6-6
    // intake 5.7-10
    // shoot 10-10,5
    // intake 10.6-14.4
    // shoot 14-15

    // if(trajstack.getActiveIdx() == 0) {
    //   if(before)
    // }

    // if (trajstack.getActiveIdx() == 0) {
    //   if (before(1.4)) {

    //   } else if (before(2.2)) {
    //     ss.queueState(State.SHOOTING);
    //   } else if (before(6.05)) {
    //     ss.autoPreset(new Setpoints(36, 0, 0.08 * i, 0.122));
    //   } else if (before(6.55)) {
    //     ss.queueState(State.SHOOTING);
    //   } else if (before(11.05)) {
    //     ss.autoPreset(new Setpoints(36, 0, 0.08 * i, 0.12));
    //   } else if (before(11.55)) {
    //     ss.queueState(State.SHOOTING);
    //   } else if (before(15)) {
    //     ss.autoPreset(new Setpoints(36, 0, 0.08 * i, 0.12));
    //   } else if (after(15)) {
    //     ss.queueState(State.IDLE);
    //   }
    // } else {
    //   if (before(1.4)) {

    //   } else if (before(2.2)) {
    //     ss.queueState(State.SHOOTING);
    //   } else if (before(6.05)) {
    //     ss.autoPreset(new Setpoints(36, 0, 0.08 * i, 0.12));
    //   } else if (before(6.55)) {
    //     ss.queueState(State.SHOOTING);
    //   } else if (before(11.05)) {
    //     ss.autoPreset(new Setpoints(36, 0, 0.08 * i, 0.12));
    //   } else if (before(11.55)) {
    //     ss.queueState(State.SHOOTING);
    //   } else if (before(15)) {
    //     ss.autoPreset(new Setpoints(36, 0, 0.08 * i, 0.12));
    //   } else if (after(15)) {
    //     ss.queueState(State.IDLE);
    //   }
    // }
  }
}
