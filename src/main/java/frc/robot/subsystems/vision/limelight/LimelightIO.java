package frc.robot.subsystems.vision.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.AutoLog;

public class LimelightIO {

  private NetworkTable limelight;

  public LimelightIO() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @AutoLog
  public static class LimelightIOInputs {
    public double tx = 0;
    public double ty = 0;
    public double tz = 0;
    public double tid = 0;
    public double ta = 0;
    public double pl = 0;
  }

  public void setPipeline(double pl) {}

  public void updateInputs(LimelightIOInputs inputs) {
    inputs.tx = limelight.getEntry("tx").getDouble(0);
    inputs.ty = limelight.getEntry("ty").getDouble(0);
    inputs.tz = limelight.getEntry("tz").getDouble(0);
    inputs.pl = limelight.getEntry("pl").getDouble(0);
    inputs.ta = limelight.getEntry("ta").getDouble(0);
    inputs.tid = limelight.getEntry("tid").getDouble(-1);
  }
}
