package frc.robot.auto.ampside;

import frc.robot.auto.AltAuto;

public class AmpSeries extends AltAuto {
  public AmpSeries() {
    super("AmpSeries", true);
    trajstack
        .appendChain()
        .append("Amp Start", false)
        .append("Amp Fork To 4", false)
        .append("Amp Fork To 5 To Strafe", false)
        .append("Strafe Run", false);

    trajstack
        .appendChain()
        .append("Amp Start", false)
        .append("Amp Fork To 5", false)
        .append("Amp Fork To 4 To Strafe", false)
        .append("Alt Strafe Run", false);
    trajstack.setActiveIdx(0);
    trajstack.generate();
  }

  @Override
  public void onInit() {
    // ss.queueSetpoints(new Setpoints(33, 0, 0.06, 0.17));
    // ss.queueState(State.TRACKING);
  }

  @Override
  public void onExecute() {
    // if (before(2.0)) {

    // } else if (between(2.0, 2.5)) {
    //   ss.queueState(State.SHOOTING);
    // } else if (between(5.0, 6.2)) {
    //   ss.queueSetpoints(new Setpoints(37, 0, 0.02, 0.15));
    //   ss.queueState(State.TRACKING);
    // } else if (between(6.2, 6.7)) {
    //   ss.queueState(State.SHOOTING);
    // } else if (between(9.0, 10.2)) {
    //   ss.queueSetpoints(new Setpoints(37, 0, -0.04, 0.155));
    //   ss.queueState(State.TRACKING);
    // } else if (between(10.2, 10.7)) {
    //   ss.queueState(State.SHOOTING);
    // } else if (between(12.0, 13.2)) {
    //   ss.queueSetpoints(new Setpoints(45, 0, -0.06, 0.125));
    //   ss.queueState(State.TRACKING);
    // } else if (between(13.2, 13.7)) {
    //   ss.queueState(State.SHOOTING);
    // } else {
    //   ss.queueSetpoints(new Setpoints(20, 8, 0, 0.05));
    //   ss.queueState(State.PRECHAMBER);
    // }
  }
}
