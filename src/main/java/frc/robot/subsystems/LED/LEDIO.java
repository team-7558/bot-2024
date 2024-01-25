package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
    @AutoLog
    public class LEDIOInputs {
        private double ledCurrentAmps = 0.0;
        private boolean isConnected = false;
    }
    
    public default void updateInputs(LEDIOInputs inputs) {}

    public default void setColours(boolean isRGB, int[] led) {}
}
