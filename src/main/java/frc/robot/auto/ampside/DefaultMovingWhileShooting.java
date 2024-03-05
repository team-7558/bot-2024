package frc.robot.auto.ampside;

import frc.robot.SS.State;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class DefaultMovingWhileShooting extends AltAuto {

  public DefaultMovingWhileShooting() {
    super("DefaultMovingWhileShooting", true);
    trajstack.appendChain().append("Default Path", false);
    trajstack.setActiveIdx(0);
  }

  @Override
  public void onInit() {}

  @Override
  public void onExecute() {
    ss.queueSetpoints(new Setpoints(20, 8, 0, 0.05));
    ss.queueState(State.SHOOTING_FROM_GROUND);
  }
}
