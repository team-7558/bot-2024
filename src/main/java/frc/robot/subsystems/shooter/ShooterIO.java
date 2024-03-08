package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double feederVolts = 0.0;
    public double feederVelRPS = 0.0;
    public double feederCurrent = 0.0;

    public double flywheelVolts = 0.0;
    public double flywheelVelRPS = 0.0;
    public double[] flywheelCurrent = new double[] {};

    public double pivotVolts = 0.0;
    public double pivotPosR = 0.0;
    public double pivotAbsPosR = 0.0;
    public double pivotVelRPS = 0.0;
    public double pivotCurrent = 0.0;

    public double turretVolts = 0.0;
    public double turretPosR = 0.0;
    public double turretAbsPosR = 0.0;
    public double turretVelRPS = 0.0;
    public double turretCurrent = 0.0;

    public boolean beamBreakInActivated = false;
    public boolean beamBreakOutActivated = false;
    public boolean turretHallEffect = false;
    public boolean pivotHallEffect = false;
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

  public default void zero() {}

  public default void toggleBrake() {}
}
