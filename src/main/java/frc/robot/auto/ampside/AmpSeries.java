package frc.robot.auto.ampside;

import frc.robot.SS.State;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class AmpSeries extends AltAuto {
  public AmpSeries() {
    super("AmpSeries", true);
    trajstack
        .appendChain()
        .append("Amp Start", false)
        .append("Amp Fork To 4 To Strafe", false)
        .append("Strafe Run", false);

    trajstack
        .appendChain()
        .append("Amp Start", false)
        .append("Amp Fork To 5 To Strafe", false)
        .append("Strafe Run", false);

    trajstack
        .appendChain()
        .append("Amp Start", false)
        .append("Amp Fork To 6 To Strafe", false)
        .append("Strafe Run", false);
    trajstack.setActiveIdx(0);
    trajstack.generate();
  }

  @Override
  public void onInit() {
    ss.autoPreset(new Setpoints(42, 0, -0.02, 0.09));
  }

  @Override
  public void onExecute() {
    if (after(1.4)) {
      ss.queueState(State.SHOOTING);
    } else if (after(2.2)) {
      ss.queueState(State.IDLE);
    }
  }
}
