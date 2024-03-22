package frc.robot.auto.ampside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

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
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    if (before(1.3)) {
      ss.autoPreset(new Setpoints(24, 0, 0, 0.138)); // fender shot
    }
    if (between(1.4, 1.5)) {
      ss.shoot();
    }

    if (between(1.6, 2.1)) {
      ss.autoPreset(new Setpoints(28, 0, 0, 0.107)); // second inner shot guess
    }

    if (between(2.2, 2.5)) {
      ss.autoShoot();
    }

    if (between(2.6, 6.1)) {
      ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083)); // op shot
    }

    if (between(6.2, 6.5)) {
      ss.autoShoot();
    }

    if (between(6.6, 10.3)) {
      ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083)); // op shot
    }

    if (between(10.4, 10.7)) {
      ss.autoShoot();
    }

    if (between(10.8, 13.2)) {
      ss.autoPreset(new Setpoints(35, 0, 0, 0.117)); // third inner shot
    }

    if (between(13.3, 13.5)) {
      ss.autoShoot();
    }

    if (between(13.6, 14.5)) {
      ss.autoPreset(new Setpoints(35, 0, i * 0.087, 0.1)); // pretty much front post
    }

    if (after(14.5)) {
      ss.autoShoot();
    }
  }
}
