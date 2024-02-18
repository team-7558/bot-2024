package frc.robot.subsystems.shooter;

public class ShooterIOIdeal implements ShooterIO {

  private double feederVel_rps = 0.0;
  private double flywheelVel_rps = 0.0;
  private double pivotPos_r = 0.0;
  private double turretPos_r = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.feederVolts_V = 0.0;
    inputs.feederVel_rps = this.feederVel_rps;
    inputs.feederCurrent_A = 0.0;
    inputs.flywheelVolts_V = 0.0;
    inputs.flywheelVel_rps = this.flywheelVel_rps;
    inputs.flywheelCurrent_A = new double[] {0.0, 0.0};
    inputs.pivotVolts_V = 0.0;
    inputs.pivotPos_r = this.pivotPos_r;
    inputs.pivotAbsPos_r = this.pivotPos_r;
    inputs.pivotVel_rps = 0.0;
    inputs.pivotCurrent_A = 0.0;
    inputs.turretVolts_V = 0.0;
    inputs.turretPos_r = this.turretPos_r;
    inputs.turretAbsPos_r = 0.0;
    inputs.turretVel_rps = 0.0;
    inputs.turretCurrent_A = 0.0;
    inputs.beamBreakActivated = false;
  }

  /** Run closed loop at the specified velocity. */
  @Override
  public void setFeederVel(double vel_rps) {
    feederVel_rps = vel_rps;
  }

  /** Run closed loop at the specified velocity. */
  @Override
  public void setFlywheelVel(double vel_rps) {
    flywheelVel_rps = vel_rps;
  }

  /** Run closed loop at the specified velocity. */
  @Override
  public void setPivotPos(double pos_r) {
    pivotPos_r = pos_r;
  }

  /** Run closed loop at the specified velocity. */
  @Override
  public void setTurretPos(double pos_r) {
    turretPos_r = pos_r;
  }

  /** Stop in open loop. */
  @Override
  public void stop() {
    feederVel_rps = 0;
    flywheelVel_rps = 0;
  }
}
