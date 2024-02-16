package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {

  @AutoLog
  public class LimelightIOInputs {
    public double tx = 0;
    public double ty = 0;
    public double ta = 0;
    public double pl = 0;
    public double tl = 0;
    // add more later that are needed
  }

  public default void updateInputs(LimelightIOInputs inputs) {}

  public default void setPipeline(double pl) {}
}
