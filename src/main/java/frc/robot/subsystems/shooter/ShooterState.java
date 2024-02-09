package frc.robot.subsystems.shooter;

public class ShooterState {
  private double pivotPos_r;
  private double turretPos_r;
  private double flywheelPos_r;

  public ShooterState(double pivotPos_r, double turretPos_r, double flywheelPos_r) {
    this.pivotPos_r = pivotPos_r;
    this.turretPos_r = turretPos_r;
    this.flywheelPos_r = flywheelPos_r;
  }

  public ShooterState() {
    this.pivotPos_r = 0;
    this.turretPos_r = 0;
    this.flywheelPos_r = 0;
  }

  public double getPivotPos() {
    return pivotPos_r;
  }

  public void setPivotPos(double pivotPos_r) {
    this.pivotPos_r = pivotPos_r;
  }

  public double getTurretPos() {
    return turretPos_r;
  }

  public void setTurretPos(double turretPos_r) {
    this.turretPos_r = turretPos_r;
  }

  public double getFlywheelVel() {
    return flywheelPos_r;
  }

  public void setFlywheelVel(double flywheelPos_r) {
    this.flywheelPos_r = flywheelPos_r;
  }
}
