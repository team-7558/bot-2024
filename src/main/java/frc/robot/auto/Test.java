package frc.robot.auto;

import frc.robot.SS.State;

public class Test extends AltAuto {

  public Test() {
    super("Test");

    trajstack.appendChain().append("3m Fwd", false);

    trajstack.setActiveIdx(0);
    trajstack.generate();
  }

  @Override
  public void onInit() {
    ss.queueState(State.IDLE);
  }

  @Override
  public void onExecute() {
    if (between(1, 4)) {
      ss.intake();
    } else if (between(5, 7)) {
      ss.chamber();
    } else if (between(7.5, 11)) {
      ss.shootPreset2();
    } else {
      ss.idle();
    }
  }
}
