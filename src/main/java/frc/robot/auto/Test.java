package frc.robot.auto;

import frc.robot.SS.State;

public class Test extends RunAltAutoCommand {

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
    System.out.println("Hmm");
    if (after(3)) {
      ss.queueState(State.TEST_2);
    }
  }
}
