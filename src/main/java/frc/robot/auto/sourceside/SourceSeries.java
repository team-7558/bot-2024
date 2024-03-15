package frc.robot.auto.sourceside;

import frc.robot.G;
import frc.robot.auto.AltAuto;

public class SourceSeries extends AltAuto {

  public SourceSeries(int idx) {
    super("SourceSeries", true);
    trajstack
        .appendChain()
        .append("SourceStart", false)
        .append("ForkTo8ToFork", false)
        .append("ForkTo7ToFork", false)
        .append("ForkTo6ToFork", false)
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
    ss.idle();
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    // ss.autoPreset(new Setpoints(40,));
  }

  @Override
  public void onExecute() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;

    if (between(1.2, 2.7)) {
      ss.chamber();
    }

    if (between(4.5, 4.8)) {
      ss.shooterSpit();
    }

    if (between(5, 5.7)) {
      ss.chamber();
    }

    if (between(7.8, 8.1)) {
      ss.shooterSpit();
    }

    if (between(10, 10.7)) {
      ss.chamber();
    }

    if (between(13.1, 13.5)) {
      ss.shooterSpit();
    }

    if (between(14.9, 15)) {
      ss.chamber();
    }

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
