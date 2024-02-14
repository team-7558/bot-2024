package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double posMeters = Elevator.MIN_HEIGHT_M;
    public double velMetersPerSecond = 0.0;
    public double accMetersPerSecond2 = 0.0;
    public double volts = 0.0;
    public double[] currents = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs Inputs) {}

  public default void setVoltage(double volts_V) {}

  public default void setVel(double vel_mps) {}

  public default void setPos(double pos_m) {}

  public default void climb(double pos_m) {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void resetPos(double pos_m) {}

  public default void stop() {}

  public default void setBrake(boolean brake) {}
}
