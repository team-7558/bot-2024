package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ShooterIOTalonFx implements ShooterIO {

  private static final double FLYWHEEL_GEAR_RATIO = 2;
  private static final double TURRET_GEAR_RATIO = 2; // TODO SET
  private static final double FEEDER_GEAR_RATIO = 1; // TODO: SET
  private static final double PIVOT_GEAR_RATIO = 1; // TODO: SET

  private final TalonFX flywheelL = new TalonFX(9);
  private final TalonFX flywheelR = new TalonFX(10);
  private final TalonFX turret = new TalonFX(5); // TODO: update
  private final TalonFX pivot = new TalonFX(7); // TODO: update
  private final TalonFX feeder = new TalonFX(8);

  private final DutyCycleEncoder tAbsEnc = new DutyCycleEncoder(2); // TODO: update
  private final DutyCycleEncoder pAbsEnc = new DutyCycleEncoder(3); // TODO: update
  // TODO: add throughbores

  private final DigitalInput beambreak = new DigitalInput(0);

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

  private final StatusSignal<Double> PVelocity = pivot.getVelocity();
  private final StatusSignal<Double> PAppliedVolts = pivot.getMotorVoltage();
  private final StatusSignal<Double> PCurrent = pivot.getStatorCurrent();
  private final StatusSignal<Double> PPosition = pivot.getPosition();

  private final StatusSignal<Double> FVelocity = feeder.getVelocity();
  private final StatusSignal<Double> FAppliedVolts = feeder.getMotorVoltage();
  private final StatusSignal<Double> FCurrent = feeder.getStatorCurrent();

  public ShooterIOTalonFx() {
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kS = 0;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;
    flywheelL.getConfigurator().apply(config);
    flywheelR.getConfigurator().apply(config);

    var turretConfig = new TalonFXConfiguration();
    turretConfig.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;
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
    feederConfig.Slot0.kV = 0.0;
    feederConfig.Slot0.kG = 0;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feederConfig.MotionMagic.MotionMagicAcceleration = 0;
    feederConfig.MotionMagic.MotionMagicCruiseVelocity = 0;

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    pivotConfig.Slot0.kP = 0;
    pivotConfig.Slot0.kI = 0;
    pivotConfig.Slot0.kD = 0;
    pivotConfig.Slot0.kS = 0;
    pivotConfig.Slot0.kV = 0;
    pivotConfig.Slot0.kA = 0;

    // TODO: tune all of that & use absolute encoder

    turret.getConfigurator().apply(turretConfig);
    feeder.getConfigurator().apply(feederConfig);
    pivot.getConfigurator().apply(pivotConfig);

    tAbsEnc.setPositionOffset(0);
    tAbsEnc.setDistancePerRotation(1);
    pAbsEnc.setPositionOffset(0);
    pAbsEnc.setDistancePerRotation(1);

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
        PVelocity,
        PAppliedVolts,
        PCurrent,
        PPosition,
        FAppliedVolts,
        FVelocity,
        FCurrent);
    // flywheelL.optimizeBusUtilization();
    // flywheelR.optimizeBusUtilization();
    // turret.optimizeBusUtilization();
    // feeder.optimizeBusUtilization();
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
    BaseStatusSignal.refreshAll(
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
        PVelocity,
        PAppliedVolts,
        PCurrent,
        PPosition,
        FAppliedVolts,
        FVelocity,
        FCurrent);
    inputs.feederVolts = FAppliedVolts.getValueAsDouble();
    inputs.feederVelRPS = FVelocity.getValueAsDouble();
    inputs.feederCurrent = FCurrent.getValueAsDouble();
    inputs.flywheelVolts =
        0.5 * (LAppliedVolts.getValueAsDouble() + RAppliedVolts.getValueAsDouble());
    inputs.flywheelVelRPS = 0.5 * (LVelocity.getValueAsDouble() + RVelocity.getValueAsDouble());
    inputs.flywheelCurrent =
        new double[] {LCurrent.getValueAsDouble(), RCurrent.getValueAsDouble()};
    inputs.pivotVolts = PAppliedVolts.getValueAsDouble();
    inputs.pivotPosR = PPosition.getValueAsDouble();
    inputs.pivotAbsPosR = 0;
    inputs.pivotVelRPS = PVelocity.getValueAsDouble();
    inputs.pivotCurrent = PCurrent.getValueAsDouble();
    inputs.turretVolts = TAppliedVolts.getValueAsDouble();
    inputs.turretPosR = TPosition.getValueAsDouble();
    inputs.turretAbsPosR = 0;
    inputs.turretVelRPS = TVelocity.getValueAsDouble();
    inputs.turretCurrent = TCurrent.getValueAsDouble();
    inputs.beamBreakActivated = beambreak.get();
  }
}
