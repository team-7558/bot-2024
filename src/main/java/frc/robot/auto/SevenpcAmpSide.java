package frc.robot.auto;

public class SevenpcAmpSide extends AltAuto {

  public SevenpcAmpSide() {
    super("SevenpcAmpSide", true);

    trajstack.appendChain().append("7pc AS #1", false).append("7pc AS #2", false).append("7pc AS #3", false).append("7pc AS #4", false).append("7pc AS #5", false).append("7pc AS #6", false).append("7pc AS #7", false);

    trajstack.setActiveIdx(0);
    trajstack.generate();
  }

  @Override
  public void onInit() {}

  @Override
  public void onExecute() {

    if(between(1,2)) {
      ss.feedFromGroundAutoPreset();
    }
  }
}
