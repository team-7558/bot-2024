package frc.robot.auto;

public class NoTurret extends AltAuto {

  public NoTurret() {
    super("NoTurret", true);
    trajstack.appendChain().append("No Turret", false);

    trajstack.setActiveIdx(0);
  }

  @Override
  public void onInit() {}

  @Override
  public void onExecute() {}
}
