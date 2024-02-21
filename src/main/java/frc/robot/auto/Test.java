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
  public void onInit() {}

  @Override
  public void onExecute() {
    if (after(3)) {
      ss.queueState(State.TEST_2);
    }
  }
}
