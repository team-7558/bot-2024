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
        .append("SourceStart", false)
        .append("ForkTo7ToFork", false)
        .append("ForkTo6ToFork", false)
        .append("ForkToSource", false);

    trajstack
        .appendChain()
        .append("AltSourceStart", false)
        .append("ForkTo8ToFork", false)
        .append("ForkTo6ToFork", false)
        .append("ForkToSource", false);

    trajstack.setActiveIdx(idx);
  }

  Setpoints firstShot = new Setpoints(24, 0, 0.115, G.isRedAlliance() ? 0.092 : 0.092);

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
        ss.autoPreset(new Setpoints(24, 0, i * 0.115, G.isRedAlliance() ? 0.092 : 0.092), true);
      } else if (before(1.8)) {
        ss.shoot();
      } else if (before(segEnd(0) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.04, 0.078), true);
      } else if (before(segEnd(0) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(1) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.04, 0.077), true);
      } else if (before(segEnd(1) + 1.28)) {
        ss.autoShoot();
      } else if (before(segEnd(2) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.04, 0.078), true);
      } else if (before(segEnd(2) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(3))) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.04, 0.078), true);
      }

      // System.out.println("0: " + segEnd(0) + " 1:" + segEnd(1) + "2: " + segEnd(2) + " 3:" +
      // segEnd(3));

      // if (before(1.2)) {
      //   ss.autoPreset(new Setpoints(35, 0, i * 0.1025, G.isRedAlliance() ? 0.092 : 0.09), false);
      // } else if (before(1.8)) {
      //   ss.shoot();
      // } else if (before(6)) {
      //   ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083), true);
      // } else if (before(6.4)) {
      //   if (velUnder(0.1)) {
      //     ss.autoShoot();
      //   }
      // } else if (before(10.3)) {
      //   ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083), true);
      // } else if (before(11)) {
      //   if (velUnder(0.1)) {
      //     ss.autoShoot();
      //   }
      // } else if (before(14.7)) {
      //   ss.autoPreset(new Setpoints(39, 0, i * 0.02, G.isRedAlliance() ? 0.083 : 0.081), true);
      // } else if (before(15)) {
      //   if (velUnder(0.1)) {
      //     ss.autoShoot();
      //   }
      // }

    } else { // Variant
      if (before(0.9)) {
        ss.autoPreset(new Setpoints(24, 0, i * 0.115, G.isRedAlliance() ? 0.092 : 0.092), true);
      } else if (before(1.8)) {
        ss.shoot();
      } else if (before(segEnd(0) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.04, 0.078), true);
      } else if (before(segEnd(0) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(1) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.04, 0.078), true);
      } else if (before(segEnd(1) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(2) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.04, 0.078), true);
      } else if (before(segEnd(2) + 1.25)) {
        ss.autoShoot();
      } else if (before(segEnd(3))) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.04, 0.078), true);
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
