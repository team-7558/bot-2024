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
        .append("SourceStart", false)
        .append("ForkTo7ToFork", false)
        .append("ForkTo8ToFork", false)
        .append("ForkTo6ToFork", false);

    trajstack.setActiveIdx(idx);
  }

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    ss.autoPreset(new Setpoints(38, 0, 0.102, 0.092));
  }

  @Override
  public void onExecute() {
    boolean critical = false;
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    if (between(0, 1.2)) {
      ss.autoPreset(new Setpoints(35, 0, i * 0.102, 0.092));
    }

    if (between(1.2, 1.8)) {
      ss.shoot();
    }

    if (between(1.9, 6)) {
      ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083));
    }

    if (between(6.1, 6.4)) {
      ss.autoShoot();
    }

    if (between(6.5, 10.3)) {
      ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083));
    }

    if (between(10.4, 11)) {
      ss.autoShoot();
    }

    if (between(11.1, 14.7)) {
      ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083));
    }

    if (between(14.8, 15)) {
      ss.autoShoot();
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
