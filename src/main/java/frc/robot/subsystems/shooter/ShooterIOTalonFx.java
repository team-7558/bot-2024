package frc.robot.subsystems.shooter;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class ShooterIOTalonFx implements ShooterIO {

  private static final double GEAR_RATIO = 2;
  // set id to zero since we dont have them yet
  private final TalonFX talonL = new TalonFX(31);
  private final TalonFX talonR = new TalonFX(32);
  private final TalonFX turret = new TalonFX(37); //TODO: update

  private final CANcoder turretCancoder = new CANcoder(38); // TODO: update

  // change to actual id's
  private final Servo leftTilt = new Servo(0);
  private final Servo rightTilt = new Servo(1);
  
  

  // getting stats from robot
  private final StatusSignal<Double> LVelocity = talonL.getVelocity();
  private final StatusSignal<Double> LAppliedVolts = talonL.getMotorVoltage();
  private final StatusSignal<Double> LCurrent = talonL.getStatorCurrent();

  private final StatusSignal<Double> RVelocity = talonR.getVelocity();
  private final StatusSignal<Double> RAppliedVolts = talonR.getMotorVoltage();
  private final StatusSignal<Double> RCurrent = talonR.getStatorCurrent();

  private final StatusSignal<Double> TVelocity = turret.getVelocity();
  private final StatusSignal<Double> TAppliedVolts = turret.getMotorVoltage();
  private final StatusSignal<Double> TCurrent = turret.getStatorCurrent();
  private final StatusSignal<Double> TPosition = turret.getPosition();

  private final StatusSignal<Double> TAbsolutePosition = turretCancoder.getAbsolutePosition();

  public ShooterIOTalonFx() {
    var config = new TalonFXConfiguration();
    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kS = 0;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;
    talonL.getConfigurator().apply(config);
    talonR.getConfigurator().apply(config);

    var turretConfig = new TalonFXConfiguration();
    config.Feedback.RotorToSensorRatio = GEAR_RATIO;
    config.CurrentLimits.StatorCurrentLimit = 30.0; //TODO: tune
    config.CurrentLimits.StatorCurrentLimitEnable = true; 
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;

    //TODO: tune all of that & use absolute encoder

    turret.getConfigurator().apply(turretConfig);

    //TODO: tune

    leftTilt.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    rightTilt.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    talonL.setControl(new Follower(talonR.getDeviceID(), false));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        LVelocity,
        LAppliedVolts,
        LCurrent,
        RVelocity,
        RAppliedVolts,
        RCurrent,
        TVelocity,
        TAppliedVolts,
        TCurrent,
        TPosition,
        TAbsolutePosition);
    talonL.optimizeBusUtilization();
    talonR.optimizeBusUtilization();
    turret.optimizeBusUtilization();

    turret.setPosition(TAbsolutePosition.getValueAsDouble());
  }

  @Override
  public void flywheelConfigurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    talonL.getConfigurator().apply(config);
    talonR.getConfigurator().apply(config);
  }

  @Override
  public void setFlywheelVelocity(double velocityRadPerSec, double ffVolts) {
    talonL.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            Shooter.ACCELERATION,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
    talonR.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            Shooter.ACCELERATION,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
    
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    talonL.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurretAngle(double angle) {
    turret.setControl(new PositionVoltage(angle));
  }

  @Override
  public void stop() {
    talonL.stopMotor();
    talonR.stopMotor();
    turret.stopMotor();
  }

  @Override
  public void setAngle(double angle) {

    // todo make it an actual irl angle
    double checkedAngle = MathUtil.clamp(angle, 0, 1);
    leftTilt.setPosition(checkedAngle);
    leftTilt.setPosition(checkedAngle);
  }

  @Override
  public void turretConfigurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    turret.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(LVelocity, LAppliedVolts, LCurrent);

    inputs.flywheelVelocityRadPerSec =
        Units.rotationsToRadians(LVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.flywheelAppliedVolts = LAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {LAppliedVolts.getValueAsDouble()};

    inputs.linearActuatorPositionLeft = leftTilt.getPosition();
    inputs.linearActuatorPositionRight = rightTilt.getPosition();

    inputs.turretPositionDeg = TPosition.getValueAsDouble();
    inputs.turretAppliedVolts = TAppliedVolts.getValueAsDouble();
    inputs.turretVelocityRadPerSec = TVelocity.getValueAsDouble();
  }
}
