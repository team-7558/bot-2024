package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double feederVolts_V = 0.0;
    public double feederVel_rps = 0.0;
    public double feederCurrent_A = 0.0;

    public double flywheelVolts_V = 0.0;
    public double flywheelVel_rps = 0.0;
    public double[] flywheelCurrent_A = new double[] {};

    public double pivotVolts_V = 0.0;
    public double pivotPos_r = 0.0;
    public double pivotAbsPos_r = 0.0;
    public double pivotVel_rps = 0.0;
    public double pivotCurrent_A = 0.0;

    public double turretVolts_V = 0.0;
    public double turretPos_r = 0.0;
    public double turretAbsPos_r = 0.0;
    public double turretVel_rps = 0.0;
    public double turretCurrent_A = 0.0;

    public boolean beamBreakActivated = false;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setFeederVolts(double volts_V) {}

  /** Run closed loop at the specified velocity. */
  public default void setFeederVel(double vel_rps) {}

  /** Run open loop at the specified voltage. */
  public default void setFlywheelVolts(double volts_V) {}

  /** Run closed loop at the specified velocity. */
  public default void setFlywheelVel(double vel_rps) {}

  /** Run open loop at the specified voltage. */
  public default void setPivotVolts(double volts_V) {}

  /** Run closed loop at the specified velocity. */
  public default void setPivotPos(double pos_r) {}

  /** Run closed loop at the specified velocity. */
  public default void setPivotVel(double vel_rps) {}

  /** Run open loop at the specified voltage. */
  public default void setTurretVolts(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setTurretPos(double pos_r) {}

  /** Run closed loop at the specified velocity. */
  public default void setTurretVel(double vel_rps) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void flywheelConfigurePID(double kP, double kI, double kD) {}

  public default void turretConfigurePID(double kP, double kI, double kD) {}
}
