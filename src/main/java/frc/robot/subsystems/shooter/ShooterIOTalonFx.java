package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.util.Util;

public class ShooterIOTalonFx implements ShooterIO {

  private static final double TMRMIN = 0, TMRMAX = 0;

  private static final double FLYWHEEL_GEAR_RATIO = 1.66;
  private static final double TURRET_GEAR_RATIO = 5.0 * 160.0 / 14.0; // TODO SET
  private static final double FEEDER_GEAR_RATIO = 1; // TODO: SET
  private static final double PIVOT_GEAR_RATIO = 81; // TODO: SET

  private static final double FLYWHEEL_VEL_SWITCH_THRESHOLD = 50.0;
  private static final double FEEDER_VEL_SWITCH_THRESHOLD = 50.0;
  private static final double TURRET_POS_SWITCH_THRESHOLD = 0.01;
  private static final double PIVOT_POS_SWITCH_THRESHOLD = 0.20;

  private final TalonFX flywheelL = new TalonFX(9);
  private final TalonFX flywheelR = new TalonFX(10);
  private final TalonFX turret = new TalonFX(5); // TODO: update
  private final TalonFX pivot = new TalonFX(7); // TODO: update
  private final TalonFX feeder = new TalonFX(8);

  private final DutyCycleEncoder tAbsEnc = new DutyCycleEncoder(1); // TODO: update
  private final DutyCycleEncoder pAbsEnc = new DutyCycleEncoder(4); // TODO: update

  private ProfiledPIDController centennialCon =
      new ProfiledPIDController(
          80, 0, 0, new TrapezoidProfile.Constraints(1.25, 0.35), Constants.globalDelta_sec);
  private ArmFeedforward centennialFF = new ArmFeedforward(0, 0.38, 0, 0);
  private double ffOffset = 0.06;

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

  // Outputs
  VoltageOut flyVolts = new VoltageOut(0, true, false, false, false);
  VelocityVoltage flyVel = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  MotionMagicVelocityVoltage flyMmVel =
      new MotionMagicVelocityVoltage(0, 0, true, 0, 1, false, false, false);

  VoltageOut feedVolts = new VoltageOut(0, true, false, false, false);
  VelocityVoltage feedVel = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  MotionMagicVelocityVoltage feedMmVel =
      new MotionMagicVelocityVoltage(0, 0, true, 0, 1, false, false, false);

  VoltageOut tVolts = new VoltageOut(0, true, false, false, false);
  PositionVoltage tPos = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  MotionMagicVoltage tMmPos = new MotionMagicVoltage(0, true, 0, 1, false, false, false);
  MotionMagicVelocityVoltage tMmVel =
      new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);

  VoltageOut pVolts = new VoltageOut(0, true, false, false, false);
  PositionVoltage pPos = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  MotionMagicVoltage pMmPos = new MotionMagicVoltage(0, true, 0, 1, false, false, false);
  MotionMagicVelocityVoltage pMmVel =
      new MotionMagicVelocityVoltage(0, 0, true, 0, 2, false, false, false);

  public ShooterIOTalonFx() {
    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Strong hold
    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kV = 0.2;

    // mm Fast ramp
    config.Slot1.kP = 0;
    config.Slot1.kI = 0;
    config.Slot1.kD = 0;
    config.Slot1.kV = 0;
    config.Slot1.kA = 0;

    config.MotionMagic.MotionMagicCruiseVelocity = 0;
    config.MotionMagic.MotionMagicAcceleration = 0;
    config.MotionMagic.MotionMagicJerk = 0;

    flywheelL.getConfigurator().apply(config);
    flywheelR.getConfigurator().apply(config);

    var feederConfig = new TalonFXConfiguration();
    feederConfig.Feedback.SensorToMechanismRatio = FEEDER_GEAR_RATIO;
    feederConfig.CurrentLimits.SupplyCurrentLimit = 30;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // mmVel
    feederConfig.Slot0.kP = 0;
    feederConfig.Slot0.kI = 0;
    feederConfig.Slot0.kD = 0;
    feederConfig.Slot0.kA = 0;
    feederConfig.Slot0.kS = 0;
    feederConfig.Slot0.kV = 0.1;

    feederConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    feederConfig.MotionMagic.MotionMagicAcceleration = 0;
    feederConfig.MotionMagic.MotionMagicJerk = 0;

    var turretConfig = new TalonFXConfiguration();
    turretConfig.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;
    turretConfig.CurrentLimits.SupplyCurrentLimit = 30.0; // TODO: tune
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // posHold
    turretConfig.Slot0.kP = 90;
    turretConfig.Slot0.kI = 0;
    turretConfig.Slot0.kD = 0;

    // mmPosMove
    turretConfig.Slot1.kP = 90;
    turretConfig.Slot1.kI = 0;
    turretConfig.Slot1.kD = 0;
    turretConfig.Slot1.kS = 0;
    turretConfig.Slot1.kV = 2.5;
    turretConfig.Slot1.kA = 0.1;

    // mmVel
    turretConfig.Slot2.kP = 0;
    turretConfig.Slot2.kI = 0;
    turretConfig.Slot2.kD = 0;
    turretConfig.Slot2.kS = 0;
    turretConfig.Slot2.kV = 0;
    turretConfig.Slot2.kA = 0;

    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 2;
    turretConfig.MotionMagic.MotionMagicAcceleration = 2;
    turretConfig.MotionMagic.MotionMagicJerk = 0;

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // posHold
    pivotConfig.Slot0.kG = 0;
    pivotConfig.Slot0.kP = 0;
    pivotConfig.Slot0.kI = 0;
    pivotConfig.Slot0.kD = 0;
    pivotConfig.Slot0.kS = 0;
    pivotConfig.Slot0.kV = 0;
    pivotConfig.Slot0.kA = 0;

    // mmPosMove
    pivotConfig.Slot1.kG = 0;
    pivotConfig.Slot1.kP = 0;
    pivotConfig.Slot1.kI = 0;
    pivotConfig.Slot1.kD = 0;
    pivotConfig.Slot1.kS = 0;
    pivotConfig.Slot1.kV = 0;
    pivotConfig.Slot1.kA = 0;

    // mmVel
    pivotConfig.Slot2.kG = 0;
    pivotConfig.Slot2.kP = 0;
    pivotConfig.Slot2.kI = 0;
    pivotConfig.Slot2.kD = 0;
    pivotConfig.Slot2.kS = 0;
    pivotConfig.Slot2.kV = 0;
    pivotConfig.Slot2.kA = 0;

    pivotConfig.MotionMagic.MotionMagicAcceleration = 0;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    pivotConfig.MotionMagic.MotionMagicJerk = 0;

    // TODO: tune all of that & use absolute encoder

    feeder.getConfigurator().apply(feederConfig);
    turret.getConfigurator().apply(turretConfig);
    pivot.getConfigurator().apply(pivotConfig);

    tAbsEnc.setPositionOffset(0.693);
    tAbsEnc.setDistancePerRotation(1);
    pAbsEnc.setPositionOffset(0.832);
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
  public void zero() {
    System.out.println(
        "tAbs Offset: " + -(tAbsEnc.getAbsolutePosition() - tAbsEnc.getPositionOffset()));
    System.out.println(
        "pAbs Offset: " + -(pAbsEnc.getAbsolutePosition() - pAbsEnc.getPositionOffset()));
    turret.setPosition(-(tAbsEnc.getAbsolutePosition() - tAbsEnc.getPositionOffset()));
    pivot.setPosition(-(pAbsEnc.getAbsolutePosition() - pAbsEnc.getPositionOffset()));
  }

  @Override
  public void stop() {
    flywheelL.stopMotor();
    flywheelR.stopMotor();
    turret.stopMotor();
    feeder.stopMotor();
    pivot.stopMotor();

    centennialCon.reset(-(pAbsEnc.getAbsolutePosition() - pAbsEnc.getPositionOffset()), 0);
  }

  @Override
  public void setFlywheelVolts(double volts) {
    double v = Util.limit(volts, 12.0);
    flywheelL.setControl(flyVolts.withOutput(v));
    flywheelR.setControl(flyVolts.withOutput(v));
  }

  @Override
  public void setFlywheelVel(double vel_rps) {
    if (Util.inRange(vel_rps - LVelocity.getValueAsDouble(), FLYWHEEL_VEL_SWITCH_THRESHOLD)) {
      flywheelL.setControl(flyVel.withVelocity(vel_rps));
      flywheelR.setControl(flyVel.withVelocity(vel_rps));
    } else {
      flywheelL.setControl(flyMmVel.withVelocity(vel_rps));
      flywheelR.setControl(flyMmVel.withVelocity(vel_rps));
    }
  }

  @Override
  public void setFeederVolts(double volts) {
    double v = Util.limit(volts, 12.0);
    feeder.setControl(feedVolts.withOutput(v));
  }

  @Override
  public void setFeederVel(double vel_rps) {
    if (Util.inRange(vel_rps - FVelocity.getValueAsDouble(), FEEDER_VEL_SWITCH_THRESHOLD)) {
      feeder.setControl(feedVel.withVelocity(vel_rps));
      feeder.setControl(feedVel.withVelocity(vel_rps));
    } else {
      feeder.setControl(feedMmVel.withVelocity(vel_rps));
      feeder.setControl(feedMmVel.withVelocity(vel_rps));
    }
  }

  /** Run open loop at the specified voltage. */
  public void setTurretVolts(double volts) {
    double v = Util.limit(volts, 12.0);
    turret.setControl(tVolts.withOutput(v));
  }

  /** Run closed loop at the specified velocity. */
  public void setTurretPos(double pos_r) {
    if (Util.inRange(pos_r - TPosition.getValueAsDouble(), TURRET_POS_SWITCH_THRESHOLD)) {
      turret.setControl(tPos.withPosition(pos_r));
      turret.setControl(tPos.withPosition(pos_r));
    } else {
      turret.setControl(tMmPos.withPosition(pos_r));
      turret.setControl(tMmPos.withPosition(pos_r));
    }
  }

  /** Run closed loop at the specified velocity. */
  public void setTurretVel(double vel_rps) {
    turret.setControl(tMmVel.withVelocity(vel_rps));
  }

  /** Run open loop at the specified voltage. */
  public void setPivotVolts(double volts_V) {
    double v = Util.limit(volts_V, 12.0);
    pivot.setControl(pVolts.withOutput(v));
  }

  /** Run closed loop at the specified velocity. */
  public void setPivotPos(double pos_r) {
    /*if (Util.inRange(pos_r - PPosition.getValueAsDouble(), PIVOT_POS_SWITCH_THRESHOLD)) {
      pivot.setControl(pPos.withPosition(pos_r));
      pivot.setControl(pPos.withPosition(pos_r));
    } else {
      pivot.setControl(pMmPos.withPosition(pos_r));
      pivot.setControl(pMmPos.withPosition(pos_r));
    }*/
    double m = -(pAbsEnc.getAbsolutePosition() - pAbsEnc.getPositionOffset());
    double output =
        centennialCon.calculate(m, pos_r)
            + centennialFF.calculate(Units.rotationsToRadians(m + ffOffset), 0);
    setPivotVolts(output);
  }

  /** Run closed loop at the specified velocity. */
  public void setPivotVel(double vel_rps) {
    pivot.setControl(pMmVel.withVelocity(vel_rps));
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
    inputs.pivotAbsPosR = -(pAbsEnc.getAbsolutePosition() - pAbsEnc.getPositionOffset());
    inputs.pivotVelRPS = PVelocity.getValueAsDouble();
    inputs.pivotCurrent = PCurrent.getValueAsDouble();
    inputs.turretVolts = TAppliedVolts.getValueAsDouble();
    inputs.turretPosR = TPosition.getValueAsDouble();
    inputs.turretAbsPosR = -(tAbsEnc.getAbsolutePosition() - tAbsEnc.getPositionOffset());
    inputs.turretVelRPS = TVelocity.getValueAsDouble();
    inputs.turretCurrent = TCurrent.getValueAsDouble();
    inputs.beamBreakActivated = beambreak.get();
  }
}
