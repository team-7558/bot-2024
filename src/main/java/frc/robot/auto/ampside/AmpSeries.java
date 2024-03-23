package frc.robot.auto.ampside;

import frc.robot.G;
import frc.robot.auto.AltAuto;
import frc.robot.subsystems.shooter.Shooter.Setpoints;
import org.littletonrobotics.junction.Logger;

public class AmpSeries extends AltAuto {
  public AmpSeries() {
    super("AmpSeries", true);
    trajstack.appendChain().append("Speakerfront8Piece", true);
    trajstack.setActiveIdx(0);
  }

  @Override
  public void onInit() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    ss.autoPreset(new Setpoints(25, 0, 0, 0.12));
  }

  @Override
  public void onExecute() {
    double i = G.isRedAlliance() ? 1.0 : -1.0;
    Logger.recordOutput("SS/LastEventMarker", getLastEventMarkerIndex());
  }
}
