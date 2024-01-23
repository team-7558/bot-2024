package frc.robot.subsystems.shooter;

public class ShooterState {
  private double hoodPosition;
  private double turretPosition;
  private double shooterVelocityRadPerSec;

  public ShooterState(double hoodPosition, double turretPosition, double shooterVelocityRadPerSec) {
    this.hoodPosition = hoodPosition;
    this.turretPosition = turretPosition;
    this.shooterVelocityRadPerSec = shooterVelocityRadPerSec;
  }

  public ShooterState() {
    this.hoodPosition = 0;
    this.turretPosition = 0;
    this.shooterVelocityRadPerSec = 0;
  }

  public double getHoodPosition() {
    return hoodPosition;
  }

  public void setHoodPosition(double hoodPosition) {
    this.hoodPosition = hoodPosition;
  }

  public double getTurretPosition() {
    return turretPosition;
  }

  public void setTurretPosition(double turretPosition) {
    this.turretPosition = turretPosition;
  }

  public double getShooterVelocity() {
    return shooterVelocityRadPerSec;
  }

  public void setShooterVelocityRadPerSec(double shooterVelocityRadPerSec) {
    this.shooterVelocityRadPerSec = shooterVelocityRadPerSec;
  }
}
