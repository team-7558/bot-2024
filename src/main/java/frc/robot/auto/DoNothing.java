package frc.robot.auto;

public class DoNothing extends AltAuto {
  public DoNothing() {
    super("DoNothing", false);
    trajstack.appendChain().append(15);
  }

  @Override
  public void onInit() {}

  @Override
  public void onExecute() {}
}
