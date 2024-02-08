package frc.robot.auto;

import frc.robot.SS.State;

public class Test extends AltAuto {

  public Test() {
    super("Test");

    addPath("LeftStart", true).generate();
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    if (after(3)) {
      System.out.println("Hmm");
      ss.queueState(State.TEST_2);
    }
  }
}
