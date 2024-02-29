package frc.robot.auto;

public class Calibration extends AltAuto {

  public Calibration() {
    super("Calibration");

    trajstack.appendChain().append("Calibration Real", false);

    trajstack.setActiveIdx(0);
    trajstack.generate();
  }

  @Override
  public void onInit() {}

  @Override
  public void onExecute() {}
}
