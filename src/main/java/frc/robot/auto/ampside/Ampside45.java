package frc.robot.auto.ampside;

import edu.wpi.first.math.util.Units;
import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;
import frc.robot.subsystems.shooter.ShotPresets;

public class Ampside45 extends AltAuto {
  public Ampside45(int index) {
    super("Ampside45", true);
    trajstack
        .appendChain()
        .append("AmpStart4", false)
        .append("AmpForkTo5ToFork", false)
        .append("AmpForkToTrap", false);
    trajstack.setActiveIdx(0);
  }

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1 : -1;
    ss.autoPreset(
        G.isRedAlliance()
            ? ShotPresets.RED_AMP_AUTO_FIRST_SHOT
            : ShotPresets.BLUE_AMP_AUTO_FIRST_SHOT,
        true);
  }

  @Override
  public void onExecute() {
    double i = G.isRedAlliance() ? 1 : -1;
    if (before(0.9)) {
      ss.autoPreset(
          G.isRedAlliance()
              ? ShotPresets.RED_AMP_AUTO_FIRST_SHOT
              : ShotPresets.BLUE_AMP_AUTO_FIRST_SHOT,
          false);
    } else if (before(1.3)) {
      ss.shoot();
    } else if (before(segEnd(0) - 0.1)) {
      ss.autoPreset(new Setpoints(45, 0, i * -0.043, 0.057), true);
    } else if (before(segEnd(0) + 1.0)) {
      ss.autoShoot();
    } else if (before(segEnd(1) - 0.1)) {
      ss.autoPreset(new Setpoints(45, 0, i * -0.043, 0.057), true);
    } else if (before(segEnd(1) + 1.0)) {
      ss.autoShoot();
    } else if (before(segEnd(2) - 0.1)) {
      ss.chamber();
    } else if (after(segEnd(2)) && before(14.4)) {
      drive.setAutolockSetpoint(
          G.isRedAlliance() ? Units.radiansToRotations(1.054) : Units.radiansToRotations(2.072));
      ss.trackTrap();
    } else if (after(14.5) && before(15)) {
      ss.shoot();
    }
  }
}
