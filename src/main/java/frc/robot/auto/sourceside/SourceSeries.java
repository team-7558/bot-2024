package frc.robot.auto.sourceside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class SourceSeries extends AltAuto {

  public SourceSeries(int idx) {
    super("SourceSeries", true);
    trajstack
        .appendChain()
        .append("SourceStart", false)
        // .append(0.1)
        .append("ForkTo7ToFork", false)
        // .append(0.1)
        .append("ForkTo6ToFork", false)
        // .append(0.1)
        .append("ForkTo5ToFork", false);

    trajstack
        .appendChain()
        .append("AltSourceStart", false)
        .append("AltForkTo5ToFork", false)
        .append("ForkTo6ToFork", false)
        .append("ForkTo5ToFork", false);

    trajstack.setActiveIdx(idx);
  }

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    ss.autoPreset(new Setpoints(38, 0, 0.1025, G.isRedAlliance() ? 0.092 : 0.09), false);
  }

  @Override
  public void onExecute() {
    boolean critical = false;
    double i = G.isRedAlliance() ? 1.0 : -1.0;

    if (trajstack.getActiveIdx() == 0) {

      if (before(1.2)) {
        ss.autoPreset(new Setpoints(35, 0, i * 0.1025, G.isRedAlliance() ? 0.092 : 0.09), false);
      } else if (before(1.8)) {
        ss.shoot();
      } else if (before(6)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083), true);
      } else if (before(6.4)) {
        if (velUnder(0.1)) {
          ss.autoShoot();
        }
      } else if (before(10.3)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083), true);
      } else if (before(11)) {
        if (velUnder(0.1)) {
          ss.autoShoot();
        }
      } else if (before(14.7)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.02, G.isRedAlliance() ? 0.083 : 0.081), true);
      } else if (before(15)) {
        if (velUnder(0.1)) {
          ss.autoShoot();
        }
      }

    } else { // Variant

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
