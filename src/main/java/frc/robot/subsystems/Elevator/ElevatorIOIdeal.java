package frc.robot.subsystems.Elevator;

import frc.robot.Constants;

public class ElevatorIOIdeal implements ElevatorIO {
  private static final double LOOP_PERIOD_SECS = Constants.globalDelta_sec;
  private double vel_radps = 0.0;
  private double volts_V = 0.0;
  private double pos_rad = 0.0;

  private int index;

  public ElevatorIOIdeal(int index) {
    this.index = index;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.volts_V = volts_V;
    inputs.vel_radps = vel_radps;
    inputs.pos_rad = pos_rad;
  }

  @Override
  public void setVel_radps(double velocity_v) {
    this.velocity_v = velocity_v;
  }

  @Override
  public void setVoltage(double volts_V) {
    this.volts_V = volts_V;
  }

  @Override
  public void setPos_rad(double pos_rad) {
    this.pos_rad = pos_rad;
  }
}
