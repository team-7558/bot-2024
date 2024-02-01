package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double pos_rad = 0.0;
    public double vel_radps = 0.0;
    public double volts_V = 0.0;
    public double[] currents_A = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs Inputs) {}

  public default void setVoltage(double volts_V) {}

  public default void setVel_radps(double velocity_v) {}

  public default void setPos_rad(double pos_rad) {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void stop() {}
}
