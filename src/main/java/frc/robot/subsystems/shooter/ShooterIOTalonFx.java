package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOTalonFx implements ShooterIO {

  private static final double FLYWHEEL_GEAR_RATIO = 2;
  private static final double TURRET_GEAR_RATIO = 2; // TODO SET
  private static final double FEEDER_GEAR_RATIO = 1; // TODO: SET

  private final TalonFX flywheelL = new TalonFX(31);
  private final TalonFX flywheelR = new TalonFX(32);
  private final TalonFX turret = new TalonFX(37); // TODO: update
  private final TalonFX pivot = new TalonFX(38); // TODO: update
  private final TalonFX feeder = new TalonFX(39);

  private final CANcoder turretCancoder = new CANcoder(38); // TODO: update
  // TODO: add throughbore

  // getting stats from robot
  private final StatusSignal<Double> LVelocity = flywheelL.getVelocity();
  private final StatusSignal<Double> LAppliedVolts = flywheelL.getMotorVoltage();
  private final StatusSignal<Double> LCurrent = flywheelL.getStatorCurrent();

  private final StatusSignal<Double> RVelocity = flywheelR.getVelocity();
  private final StatusSignal<Double> RAppliedVolts = flywheelR.getMotorVoltage();
  private final StatusSignal<Double> RCurrent = flywheelR.getStatorCurrent();

  private final StatusSignal<Double> TVelocity = turret.getVelocity();
  private final StatusSignal<Double> TAppliedVolts = turret.getMotorVoltage();
  private final StatusSignal<Double> TCurrent = turret.getStatorCurrent();
  private final StatusSignal<Double> TPosition = turret.getPosition();

  private final StatusSignal<Double> feederVelocity = feeder.getVelocity();
  private final StatusSignal<Double> feederAppliedVolts = feeder.getMotorVoltage();
  private final StatusSignal<Double> feederCurrent = feeder.getStatorCurrent();
  private final StatusSignal<Double> feederPosition = feeder.getPosition();

  private final StatusSignal<Double> TAbsolutePosition = turretCancoder.getAbsolutePosition();

  public ShooterIOTalonFx() {
    var config = new TalonFXConfiguration();
    config.Feedback.RotorToSensorRatio = FLYWHEEL_GEAR_RATIO;
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kS = 0;
    config.Slot0.kV = 0.5;
    config.Slot0.kA = 0;
    flywheelL.getConfigurator().apply(config);
    flywheelR.getConfigurator().apply(config);

    var turretConfig = new TalonFXConfiguration();
    turretConfig.Feedback.RotorToSensorRatio = TURRET_GEAR_RATIO;
    turretConfig.CurrentLimits.StatorCurrentLimit = 30.0; // TODO: tune
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    turretConfig.Slot0.kP = 0;
    turretConfig.Slot0.kI = 0;
    turretConfig.Slot0.kD = 0;
    turretConfig.MotionMagic.MotionMagicAcceleration = 0;
    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    turretConfig.MotionMagic.MotionMagicJerk = 0;

    var feederConfig = new TalonFXConfiguration();
    feederConfig.Feedback.SensorToMechanismRatio = FEEDER_GEAR_RATIO;
    feederConfig.Slot0.kP = 0;
    feederConfig.Slot0.kI = 0;
    feederConfig.Slot0.kD = 0;
    feederConfig.Slot0.kA = 0;
    feederConfig.Slot0.kS = 0;
    feederConfig.Slot0.kV = 0.5;
    feederConfig.Slot0.kG = 0;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfig.MotionMagic.MotionMagicAcceleration = 0;
    feederConfig.MotionMagic.MotionMagicCruiseVelocity = 0;

    // TODO: tune all of that & use absolute encoder

    turret.getConfigurator().apply(turretConfig);
    feeder.getConfigurator().apply(feederConfig);

    // TODO: tune

    // talonL.setControl(new Follower(talonR.getDeviceID(), false));

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
    flywheelL.optimizeBusUtilization();
    flywheelR.optimizeBusUtilization();
    turret.optimizeBusUtilization();
    feeder.optimizeBusUtilization();

    turret.setPosition(TAbsolutePosition.getValueAsDouble());
  }

  @Override
  public void flywheelConfigurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    flywheelL.getConfigurator().apply(config);
    flywheelR.getConfigurator().apply(config);
  }

  @Override
  public void setFlywheelVel(double vel_rps) {
    flywheelL.setControl(
        new VelocityVoltage(vel_rps, Shooter.ACCELERATION, true, 0.0, 0, false, false, false));
    flywheelR.setControl(
        new VelocityVoltage(vel_rps, Shooter.ACCELERATION, true, 0, 0, false, false, false));
  }

  @Override
  public void setFlywheelVolts(double volts) {
    flywheelL.setControl(new VoltageOut(volts));
    flywheelR.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    flywheelL.stopMotor();
    flywheelR.stopMotor();
    turret.stopMotor();
    feeder.stopMotor();
  }

  @Override
  public void setFeederVolts(double volts) {
    feeder.setControl(new VoltageOut(volts));
  }

  @Override
  public void setFeederVel(double vel_rps) {
    feeder.setControl(new VelocityVoltage(vel_rps));
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
    // BaseStatusSignal.refreshAll(LVelocity, LAppliedVolts, LCurrent);

    // inputs.flywheelVelocityRadPerSec = Units.rotationsToRadians(LVelocity.getValueAsDouble());
    // inputs.flywheelAppliedVolts = LAppliedVolts.getValueAsDouble();
    // inputs.currentAmps = new double[] {LAppliedVolts.getValueAsDouble()};

    // inputs.linearActuatorPositionLeft = leftTilt.getPosition();
    // inputs.linearActuatorPositionRight = rightTilt.getPosition();

    // inputs.turretPositionDeg = TPosition.getValueAsDouble();
    // inputs.turretAppliedVolts = TAppliedVolts.getValueAsDouble();
    // inputs.turretVelocityRadPerSec = Units.rotationsToRadians(TVelocity.getValueAsDouble());

    // inputs.feederCurrent = new double[] {feederCurrent.getValueAsDouble()};
    // inputs.feederVelocity =
    //     Units.rotationsToRadians(inputs.feederVelocity = feederVelocity.getValueAsDouble());
    // inputs.feederPosition = feederPosition.getValueAsDouble();
    // inputs.feederVoltage = feederAppliedVolts.getValueAsDouble();
  }
}
