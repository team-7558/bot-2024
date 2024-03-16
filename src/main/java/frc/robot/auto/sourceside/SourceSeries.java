package frc.robot.auto.sourceside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class SourceSeries extends AltAuto {

  public SourceSeries(int idx) {
    super("SourceSeries", true);
    trajstack
        .appendChain()
        .append(0.5)
        .append("SourceStart", false)
        .append("ForkTo8ToFork", false)
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
    ss.autoPreset(new Setpoints(38, 0, i * 0.082, 0.068));
  }

  @Override
  public void onExecute() {
    boolean critical = false;
    double i = G.isRedAlliance() ? 1.0 : -1.0;

    if (between(2.1, 2.3)) {
      ss.shoot();
      critical = true;
    }

    if (between(2.4, 5.2)) {
      ss.autoPreset(new Setpoints(39, 0, i * 0.105, 0.064));
    }


    if (between(5.3, 5.6)) {
      ss.shoot();
      critical = true;
    }

    if (between(5.6, 5.7)) {
      ss.shooterSpit();
    }

    if (between(5.7, 8.7)) {
      ss.autoPreset(new Setpoints(40, 0, i * 0.105, 0.064));
    }

    if (between(8.8, 9.4)) {
      ss.shoot();
      critical = true;
    }

    if (between(9.5, 14.5)) {
      ss.autoPreset(new Setpoints(40, 0, i * 0.02, 0.085));
    }

    if (between(14.6, 15)) {
      ss.shoot();
      critical = true;
    }

    if (between(15.2, 15)) {
      ss.chamber();
    }

    org.littletonrobotics.junction.Logger.getInstance()
        .recordOutput("Drive/ShootingPoint", critical);

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
