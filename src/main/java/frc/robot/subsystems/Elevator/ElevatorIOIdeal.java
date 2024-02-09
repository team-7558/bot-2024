package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public class ElevatorIOIdeal implements ElevatorIO {
  private double vel_mps = 0.0;
  private double volts_V = 0.0;
  private double pos_m = Elevator.MIN_HEIGHT_M;

  public ElevatorIOIdeal() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.volts_V = volts_V;
    inputs.vel_mps = vel_mps;
    inputs.pos_m = pos_m;
  }

  @Override
  public void setVel(double velocity_mps) {
    this.vel_mps = velocity_mps;
    this.pos_m += velocity_mps * Constants.globalDelta_sec;
  }

  @Override
  public void setVoltage(double volts_V) {
    this.volts_V = volts_V;
  }

  @Override
  public void setPos(double pos_m) {
    this.vel_mps = 0;
    this.volts_V = 0;
    this.pos_m = pos_m;
  }
}
