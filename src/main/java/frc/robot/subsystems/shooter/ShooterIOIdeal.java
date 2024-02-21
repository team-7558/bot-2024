package frc.robot.subsystems.shooter;

public class ShooterIOIdeal implements ShooterIO {

  private double feederVel_rps = 0.0;
  private double flywheelVel_rps = 0.0;
  private double pivotPos_r = 0.0;
  private double turretPos_r = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.feederVolts = 0.0;
    inputs.feederVelRPS = this.feederVel_rps;
    inputs.feederCurrent = 0.0;
    inputs.flywheelVolts = 0.0;
    inputs.flywheelVelRPS = this.flywheelVel_rps;
    inputs.flywheelCurrent = new double[] {0.0, 0.0};
    inputs.pivotVolts = 0.0;
    inputs.pivotPosR = this.pivotPos_r;
    inputs.pivotAbsPosR = this.pivotPos_r;
    inputs.pivotVelRPS = 0.0;
    inputs.pivotCurrent = 0.0;
    inputs.turretVolts = 0.0;
    inputs.turretPosR = this.turretPos_r;
    inputs.turretAbsPosR = 0.0;
    inputs.turretVelRPS = 0.0;
    inputs.turretCurrent = 0.0;
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
