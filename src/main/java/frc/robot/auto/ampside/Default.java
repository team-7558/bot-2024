package frc.robot.auto.ampside;

import frc.robot.SS.State;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class Default extends AltAuto {

  public Default() {
    super("DefaultAmpside", true);
    trajstack.appendChain().append("Default Ampside", false);
    trajstack.setActiveIdx(0);

    trajstack.generate();
  }

  @Override
  public void onInit() {
    ss.queueState(State.IDLE);
  }

  @Override
  public void onExecute() {

    if (between(0.3, 1.9)) {
      Shooter.getInstance().queueSetpoints(new Setpoints(30, 0, 0.1, 0.1));
      ss.shoot();
    } else if (between(2.1, 2.7)) {
      Shooter.getInstance().setCurrentState(Shooter.getInstance().BEING_FED);
      Shooter.getInstance().queueSetpoints(new Setpoints(0, 0, 0.1, 0));
    } else if (between(2.9, 3.3)) {
      ss.feedFromGroundAutoPreset();
    } else if (after(3.7)) {
      Shooter.getInstance().queueSetpoints(new Setpoints(30, 0, 0.1, 0.1));
      ss.shoot();
    } else {
      ss.idle();
    }
  }
}
