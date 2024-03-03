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

    // if(between(1,1.30)) {
    //   ss.shoot();
    // }
    // if(between(2.85, 3.15)) {
    //   ss.feedFromGroundAutoPreset();
    // }
    // if(between(4,4.5)) {
    //   ss.shoot();
    // }
    // if(between(5.5, 5.8)) {
    //   ss.feedFromGroundAutoPreset();
    // }
    // if(between(6.75,7.05)) {
    //   ss.shoot();
    // }
    // if(between(8.3, 8.6)) {
    //   ss.feedFromGroundAutoPreset();
    // }
    // if(between(11.5,11.8)) {
    //   ss.shoot();
    // }
    // if(between(12.5, 15.7  )) {
    //   ss.feedFromGroundAutoPreset();
    // }
    // if(between(12.5, 15.7)) {
    //   ss.feedFromGroundAutoPreset();
    // }
  }
}
