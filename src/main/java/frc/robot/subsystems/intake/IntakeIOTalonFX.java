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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.Util;

public class IntakeIOTalonFX implements IntakeIO {
  private static final double GEAR_RATIO = 0;

  private final TalonFX topMotor = new TalonFX(2); // Not gunna be 0 1 (cameron told me to add that)
  private final TalonFX middleMotor = new TalonFX(1);
  private final TalonFX bottomMotor = new TalonFX(0);
  private final DigitalInput bottomSensor = new DigitalInput(0);
  private final DigitalInput topSensor = new DigitalInput(1);

  private final StatusSignal<Double> bottomPosition = bottomMotor.getPosition();
  private final StatusSignal<Double> bottomVelocity = bottomMotor.getVelocity();
  private final StatusSignal<Double> bottomAppliedVolts = bottomMotor.getMotorVoltage();
  private final StatusSignal<Double> bottomCurrent = bottomMotor.getStatorCurrent();

  private final StatusSignal<Double> middlePosition = middleMotor.getPosition();
  private final StatusSignal<Double> middleVelocity = middleMotor.getVelocity();
  private final StatusSignal<Double> middleAppliedVolts = middleMotor.getMotorVoltage();
  private final StatusSignal<Double> middleCurrent = middleMotor.getStatorCurrent();

  private final StatusSignal<Double> topPosition = topMotor.getPosition();
  private final StatusSignal<Double> topVelocity = topMotor.getVelocity();
  private final StatusSignal<Double> topAppliedVolts = topMotor.getMotorVoltage();
  private final StatusSignal<Double> topCurrent = topMotor.getStatorCurrent();



  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomMotor.getConfigurator().apply(config);
    middleMotor.getConfigurator().apply(config);
    topMotor.getConfigurator().apply(config);

    middleMotor.setControl(new Follower(bottomMotor.getDeviceID(), false));
    topMotor.setControl(new Follower(bottomMotor.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, bottomPosition, bottomVelocity, bottomAppliedVolts, bottomCurrent, middlePosition, middleVelocity, middleAppliedVolts, middleCurrent, topPosition, topVelocity, topAppliedVolts, topCurrent);
    topMotor.optimizeBusUtilization();
    middleMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        bottomPosition, bottomVelocity, bottomAppliedVolts, bottomCurrent, middlePosition, middleVelocity, middleAppliedVolts, middleCurrent, topPosition, topVelocity, topAppliedVolts, topCurrent);
    // inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.beamBreakActivatedBottom = bottomSensor.get();
    inputs.beamBreakActivatedTop = topSensor.get();
    inputs.intakeVelocityRadPerSec =
        Units.rotationsToRadians(bottomVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.intakeAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps =
        new double[] {bottomCurrent.getValueAsDouble(), middleCurrent.getValueAsDouble(), topCurrent.getValueAsDouble()};
  }

  @Override
  public void setIntakeSpeed(double speed) {
    bottomMotor.setControl(
        new DutyCycleOut(speed));
  }

  @Override
  public void setIntakeVoltage(double volts) {
    bottomMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setIntakeVelocity(double velocityRadPerSec) {
    bottomMotor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setElevatorSpeed(double speed) {
    middleMotor.setControl(
        new DutyCycleOut(speed));
  }

  @Override
  public void setElevatorVoltage(double volts) {
    middleMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setElevatorVelocity(double velocityRadPerSec) {
    middleMotor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setDirectionSpeed(double speed) { // speed value would be -12/12
    double result = Util.limit(speed * 12.0, -12.0, 12.0);
    topMotor.setControl(
        new VoltageOut(result));
  }

  @Override
  public void setDirectionVoltage(double volts) {
    topMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDirectionVelocity(double velocityRadPerSec) {
    topMotor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec)));
  }


  @Override
  public void stop() {
    bottomMotor.stopMotor();
  }

 
}