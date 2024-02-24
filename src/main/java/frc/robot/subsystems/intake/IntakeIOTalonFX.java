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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFX implements IntakeIO {
  private static final double GEAR_RATIO = 1;

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
  private final VelocityVoltage bottomVelOut = new VelocityVoltage(0);
  private final DutyCycleOut topDCOut = new DutyCycleOut(0);
  private final VoltageOut topVOut = new VoltageOut(0);
  private final VelocityVoltage topVelOut = new VelocityVoltage(0);

  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    bottomMotor.getConfigurator().apply(config);
    topMotor.getConfigurator().apply(config);

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
    inputs.intakeVelocityRadPerSec =
        Units.rotationsToRadians(bottomVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.intakeAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {bottomCurrent.getValueAsDouble(), topCurrent.getValueAsDouble()};

    inputs.directionVelocityRadPerSec =
        Units.rotationsToRadians(topVelocity.getValueAsDouble()) / GEAR_RATIO;
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
  public void setIntakeVelocity(double velocityRadPerSec) {
    bottomMotor.setControl(bottomVelOut.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
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
  public void setDirectionVelocity(double velocityRadPerSec) {
    topMotor.setControl(topVelOut.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }
}
