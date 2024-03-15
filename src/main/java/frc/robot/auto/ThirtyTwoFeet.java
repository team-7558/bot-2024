package frc.robot.auto;

public class ThirtyTwoFeet extends AltAuto {

  public ThirtyTwoFeet() {
    super("32ft", true);
    trajstack.appendChain().append("thirtytwo", false);

    trajstack.setActiveIdx(0);
  }

  @Override
  public void onInit() {}

  @Override
  public void onExecute() {}
}
