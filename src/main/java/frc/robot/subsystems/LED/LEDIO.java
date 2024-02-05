package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public class LEDIOInputs {
    public double ledCurrentAmps = 0.0;
    public boolean isConnected = false;
  }

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void setColours(boolean isRGB, int[] led) {}
  public default void setColours(boolean isRGB, int[] led) {}
}
