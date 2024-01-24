// Copyright 2021-2024 FRC 6328
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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIO2023 implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final DutyCycleEncoder absEncoder;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final DoubleSupplier turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  private final VoltageOut driveOut_V, turnOut_V;
  private final VelocityVoltage driveOut_rotps;
  private final PositionVoltage turnOut_rot;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  // private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  // private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  // Gear ratios for SDS MK4 L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 12.8 / 1.0;

  private boolean isDriveMotorInverted = true;
  private final boolean isTurnMotorInverted = false;
  private final boolean isAbsEncoderInverted = false;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIO2023(int index) {
    var driveConfig = new TalonFXConfiguration();
    var turnConfig = new TalonFXConfiguration();

    switch (index) {
      case Drive.FL:
        driveTalon = new TalonFX(2);
        turnTalon = new TalonFX(6);
        absEncoder = new DutyCycleEncoder(0);
        // absoluteEncoderOffset = Rotation2d.fromRadians(0); // TODO: TUNE
        absoluteEncoderOffset = Rotation2d.fromRadians(-2.710 + Math.PI);
        break;
      case Drive.FR:
        driveTalon = new TalonFX(3);
        turnTalon = new TalonFX(7);
        absEncoder = new DutyCycleEncoder(2);
        // absoluteEncoderOffset = Rotation2d.fromRadians(0); // TODO: TUNE
        absoluteEncoderOffset = Rotation2d.fromRadians(0.291);
        isDriveMotorInverted = false;
        break;
      case Drive.BL:
        driveTalon = new TalonFX(1);
        turnTalon = new TalonFX(5);
        absEncoder = new DutyCycleEncoder(1);
        // absoluteEncoderOffset = Rotation2d.fromRadians(0); // TODO: TUNE
        absoluteEncoderOffset = Rotation2d.fromRadians(2.741 - Math.PI);
        break;
      case Drive.BR:
        driveTalon = new TalonFX(4);
        turnTalon = new TalonFX(8);
        absEncoder = new DutyCycleEncoder(3);
        // absoluteEncoderOffset = Rotation2d.fromRadians(0); // TODO: TUNE
        absoluteEncoderOffset = Rotation2d.fromRadians(-2.124 + Math.PI);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    absEncoder.setDutyCycleRange(1.0 / 4096.0, 4095.0 / 4096.0); // Might need to be 0, 4096

    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    driveConfig.Slot0.kP = 0.0; // 2.402346
    driveConfig.Slot0.kI = 0;
    driveConfig.Slot0.kD = 0;
    driveConfig.Slot0.kS = 0.0;
    driveConfig.Slot0.kV = 0.5;
    driveConfig.Slot0.kA = 0.0;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;
    turnConfig.Slot0.kP = 14.41407;
    turnConfig.Slot0.kI = 0.0;
    turnConfig.Slot0.kD = 0.288281;
    turnConfig.Slot0.kS = 0.0;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    turnTalon.setPosition(
        Units.radiansToRotations(
            absEncoder.getAbsolutePosition() * (isAbsEncoderInverted ? -1.0 : 1.0)
                - absoluteEncoderOffset.getRadians()));

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition =
        () -> absEncoder.getAbsolutePosition() * (isAbsEncoderInverted ? -1.0 : 1.0);
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    // driveTalon.optimizeBusUtilization();
    // turnTalon.optimizeBusUtilization();

    driveOut_V = new VoltageOut(0);
    turnOut_V = new VoltageOut(0);

    driveOut_rotps = new VelocityVoltage(0);
    turnOut_rot = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getAsDouble()).minus(absoluteEncoderOffset);
    inputs.turnPositionRad = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(driveOut_V.withOutput(volts));
  }

  @Override
  public void setDriveVelocity(double velocity) {
    double vel_mps = MathUtil.clamp(velocity, -Drive.MAX_LINEAR_SPEED, Drive.MAX_LINEAR_SPEED);

    double vel_radps = vel_mps / Module.WHEEL_RADIUS;
    double vel_rotps = Units.radiansToRotations(vel_radps);

    driveTalon.setControl(driveOut_rotps.withVelocity(vel_rotps));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(turnOut_V.withOutput(volts));
  }

  @Override
  public void setTurnAngle(double rad) {
    turnTalon.setControl(turnOut_rot.withPosition(Units.radiansToRotations(rad)));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isDriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }
}
