package frc.robot.auto.ampside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class DefaultMovingWhileShooting extends AltAuto {

  public DefaultMovingWhileShooting() {
    super("DefaultMovingWhileShooting", true);
    trajstack.appendChain().append(0.3).append("Default Path", false);
    trajstack.setActiveIdx(0);
    // trajstack.generate();
  }

  @Override
  public void onInit() {
    ss.autoPresetNoTurret(new Setpoints(26, 0, 0, 0.14));
  }

  @Override
  public void onExecute() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;

    if (between(0.3, 0.8)) {
      ss.autoPresetNoTurret(new Setpoints(26, 0, 0, 0.14));
    }

    if (between(0.9, 1.4)) {
      ss.shoot();
    }

    if (between(1.5, 5.9)) {
      ss.autoPresetNoTurret(new Setpoints(33, 0, 0, 0.12));
    }
    if (between(6.2, 6.9)) {
      ss.shoot();
    }

    if (between(6.9, 8.6)) {
      ss.autoPresetNoTurret(new Setpoints(32, 0, 0, 0.12));
    }

    if (between(8.5, 8.7)) {
      ss.shoot();
    }

    if (between(8.8, 12.5)) {
      ss.autoPresetNoTurret(new Setpoints(30, 0, 0, 0.089));
    }

    if (between(12.6, 13)) {
      ss.shoot();
    }

    if (between(14, 15)) {
      ss.chamber();
    }
  }
}
