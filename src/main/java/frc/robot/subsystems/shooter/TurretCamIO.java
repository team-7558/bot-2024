package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface TurretCamIO {
  @AutoLog
  public static class TurretCamIOInputs {
    public boolean connected = false;
    public boolean tv = false;
    public double latency = 0.0;
    public int ids = 0;
    public double tx = 0.0;
    public double ty = 0.0;
    public double ta = 0.0;
    public double tid = 0.0;
  }

  public default void updateInputs(TurretCamIOInputs inputs) {}

  public default void setPipeline(Pipeline pipeline) {}

  public default void snapshot() {}

  public default void setLEDs(LEDStatus on) {}

  enum LEDStatus {
    OFF,
    LOW,
    HI
  }

  enum Pipeline {
    NEAR,
    FAR,
    TRAP
  }
}
