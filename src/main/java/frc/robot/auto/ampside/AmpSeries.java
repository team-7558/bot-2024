package frc.robot.auto.ampside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class AmpSeries extends AltAuto {
  public AmpSeries(int idx) {
    super("AmpSeries", true);
    trajstack
        .appendChain()
        .append("Ampside Start", false)
        .append("AmpForkTo5ToFork", false)
        .append("AmpForkTo6ToOp", false)
        .append("ForkTo7ToFork", false);

    trajstack
        .appendChain()
        .append("Alt Ampside Start", false)
        .append("AmpForkTo4ToFork", false)
        .append("AmpForkTo6ToOp", false)
        .append("ForkTo7ToFork", false);

    trajstack.setActiveIdx(idx);
  }

  Setpoints firstShot = new Setpoints(24, 0, -0.055, G.isRedAlliance() ? 0.108 : 0.108);

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    ss.autoPreset(new Setpoints(24, 0, i * -0.055, G.isRedAlliance() ? 0.108 : 0.107), false);
    // ss.queueSetpointsLive();
  }

  @Override
  public void onExecute() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    // ss.queueSetpointsLive();

    if (trajstack.getActiveIdx() == 0) {

      if (before(0.9)) {
        ss.autoPreset(new Setpoints(24, 0, i * -0.055, G.isRedAlliance() ? 0.108 : 0.107), false);
      } else if (before(1.3)) {
        ss.shoot();
      } else if (before(segEnd(0) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * -0.02, G.isRedAlliance() ? 0.083 : 0.082), false);
      } else if (before(segEnd(0) + 1.0)) {
        ss.autoShoot();
      } else if (before(segEnd(1) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.02, G.isRedAlliance() ? 0.073 : 0.072), false);
      } else if (before(segEnd(1) + 1.0)) {
        ss.autoShoot();
      } else if (before(segEnd(2) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.02, G.isRedAlliance() ? 0.073 : 0.072), false);
      } else if (before(segEnd(2) + 1.0)) {
        ss.autoShoot();
      } else if (before(segEnd(3))) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.02, 0.083), false);
      }
    } else { // Variant
      if (before(0.9)) {
        ss.autoPreset(new Setpoints(24, 0, i * -0.055, G.isRedAlliance() ? 0.108 : 0.107), false);
      } else if (before(1.3)) {
        ss.shoot();
      } else if (before(segEnd(0) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * -0.02, G.isRedAlliance() ? 0.073 : 0.072), false);
      } else if (before(segEnd(0) + 1.0)) {
        ss.autoShoot();
      } else if (before(segEnd(1) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.02, G.isRedAlliance() ? 0.073 : 0.072), false);
      } else if (before(segEnd(1) + 1.0)) {
        ss.autoShoot();
      } else if (before(segEnd(2) - 0.1)) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.02, G.isRedAlliance() ? 0.083 : 0.082), false);
      } else if (before(segEnd(2) + 1.0)) {
        ss.autoShoot();
      } else if (before(segEnd(3))) {
        ss.autoPreset(new Setpoints(39, 0, i * 0.02, G.isRedAlliance() ? 0.083 : 0.082), false);
      }
    }
  }
}
