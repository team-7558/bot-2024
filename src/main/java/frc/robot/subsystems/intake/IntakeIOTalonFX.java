// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFX implements IntakeIO {
  private static final double TOP_GEAR_RATIO = 16.0 / 24.0;
  private static final double BOTTOM_GEAR_RATIO = 24.0 / 36.0;

  private static final double TOP_RADIUS_M = Units.inchesToMeters(0.605);
  private static final double BOTTOM_RADIUS_M = Units.inchesToMeters(0.605);

  private static final double TOP_RPS_TO_MPS = TOP_RADIUS_M * Math.PI * 2.0;
  private static final double BOTTOM_RPS_TO_MPS = BOTTOM_RADIUS_M * Math.PI * 2.0;

  private final TalonFX topMotor = new TalonFX(3); // Not gunna be 0 1 (cameron told me to add that)
  private final TalonFX bottomMotor = new TalonFX(2);
  private final DigitalInput beambreak = new DigitalInput(7);

  private final StatusSignal<Double> bottomVelocity = bottomMotor.getVelocity();
  private final StatusSignal<Double> bottomAppliedVolts = bottomMotor.getMotorVoltage();
  private final StatusSignal<Double> bottomCurrent = bottomMotor.getSupplyCurrent();

  private final StatusSignal<Double> topVelocity = topMotor.getVelocity();
  private final StatusSignal<Double> topAppliedVolts = topMotor.getMotorVoltage();
  private final StatusSignal<Double> topCurrent = topMotor.getSupplyCurrent();

  private final DutyCycleOut bottomDCOut = new DutyCycleOut(0);
  private final VoltageOut bottomVOut = new VoltageOut(0);
  private final MotionMagicVelocityVoltage bottomVelOut =
      new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final DutyCycleOut topDCOut = new DutyCycleOut(0);
  private final VoltageOut topVOut = new VoltageOut(0);
  private final MotionMagicVelocityVoltage topVelOut =
      new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);

  private boolean isBraked = true;

  public IntakeIOTalonFX() {
    var bottomconfig = new TalonFXConfiguration();
    bottomconfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    bottomconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    bottomconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    bottomconfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    bottomconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    bottomconfig.Feedback.SensorToMechanismRatio = BOTTOM_GEAR_RATIO;

    bottomconfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
    bottomconfig.MotionMagic.MotionMagicAcceleration = 0.5;
    bottomconfig.MotionMagic.MotionMagicJerk = 0;

    bottomconfig.Slot0.kV = 0;
    bottomconfig.Slot0.kA = 0;
    bottomconfig.Slot0.kP = 0;
    bottomconfig.Slot0.kD = 0;
    bottomMotor.getConfigurator().apply(bottomconfig);

    var topconfig = new TalonFXConfiguration();
    topconfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    topconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    topconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    topconfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    topconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    topconfig.Feedback.SensorToMechanismRatio = TOP_GEAR_RATIO;

    topconfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
    topconfig.MotionMagic.MotionMagicAcceleration = 0.5;
    topconfig.MotionMagic.MotionMagicJerk = 0;

    topconfig.Slot0.kV = 0;
    topconfig.Slot0.kA = 0;
    topconfig.Slot0.kP = 0;
    topconfig.Slot0.kD = 0;
    topMotor.getConfigurator().apply(topconfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        bottomVelocity,
        bottomAppliedVolts,
        bottomCurrent,
        topVelocity,
        topAppliedVolts,
        topCurrent);
    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        bottomVelocity,
        bottomAppliedVolts,
        bottomCurrent,
        topVelocity,
        topAppliedVolts,
        topCurrent);
    inputs.beamBreakActivated = beambreak.get();
    inputs.intakeVelocityMPS = bottomVelocity.getValueAsDouble() * BOTTOM_RPS_TO_MPS;
    inputs.intakeAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {bottomCurrent.getValueAsDouble(), topCurrent.getValueAsDouble()};

    inputs.directionVelocityMPS = topVelocity.getValueAsDouble() * TOP_RPS_TO_MPS;
    inputs.directionAppliedVolts = topAppliedVolts.getValueAsDouble();
  }

  @Override
  public void setIntakeSpeed(double speed) {
    bottomMotor.setControl(bottomDCOut.withOutput(speed));
  }

  @Override
  public void setIntakeVoltage(double volts) {
    bottomMotor.setControl(bottomVOut.withOutput(volts));
  }

  @Override
  public void setIntakeVelocity(double velMPS) {
    bottomMotor.setControl(bottomVelOut.withVelocity(velMPS / BOTTOM_RPS_TO_MPS));
  }

  @Override
  public void setDirectionSpeed(double speed) { // speed value would be -1 to 1
    topMotor.setControl(topDCOut.withOutput(speed));
  }

  @Override
  public void setDirectionVoltage(double volts) {
    topMotor.setControl(topVOut.withOutput(volts));
  }

  @Override
  public void setDirectionVelocity(double velMPS) {
    topMotor.setControl(topVelOut.withVelocity(velMPS / TOP_RPS_TO_MPS));
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  @Override
  public void toggleBrake() {
    var cfg = new MotorOutputConfigs();
    cfg.Inverted = InvertedValue.CounterClockwise_Positive;
    cfg.NeutralMode = isBraked ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    bottomMotor.getConfigurator().apply(cfg);
    topMotor.getConfigurator().apply(cfg);
  }
}
