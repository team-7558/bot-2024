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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Ideal implementation of module IO.
 *
 * <p>Tracks any setpoint without any amount of error, acts as an "ideal" module
 */
public class ModuleIOIdeal implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = Constants.globalDelta_sec;

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);

  private double drivePos_r = 0.0;
  private double driveVel_mps = 0.0;
  private double driveVolts_V = 0.0;

  private double turnPos_r = 0.0;
  private double turnVel_rps = 0.0;
  private double turnVolts_V = 0.0;

  private int index;

  public ModuleIOIdeal(int index) {
    this.index = index;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePos_r = drivePos_r;
    inputs.driveVel_mps = driveVel_mps;
    inputs.driveVolts_V = driveVolts_V;
    inputs.driveCurrent_A = new double[] {0};

    inputs.turnAbsPos_rot2d = new Rotation2d(turnPos_r).plus(turnAbsoluteInitPosition);
    inputs.turnPos_rot2d = new Rotation2d(turnPos_r);
    inputs.turnVel_rps = turnVel_rps;
    inputs.turnVel_rps = turnVolts_V;
    inputs.turnCurrent_A = new double[] {Math.abs(0)};

    inputs.odometryDrivePos_r = new double[] {inputs.drivePos_r};
    inputs.odometryTurnPos_rot2d = new Rotation2d[] {inputs.turnPos_rot2d};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveVolts_V = MathUtil.clamp(volts, -12.0, 12.0);
    setDriveVelocity((driveVolts_V / 12.0) * Drive.MAX_LINEAR_SPEED_MPS);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnVolts_V = MathUtil.clamp(volts, -12.0, 12.0);
    turnVel_rps = (turnVolts_V / 12.0) * 4;
    turnPos_r += turnVel_rps * LOOP_PERIOD_SECS;
  }

  @Override
  public void setDriveVelocity(double velocity) {
    driveVel_mps = MathUtil.clamp(velocity, -Drive.MAX_LINEAR_SPEED_MPS, Drive.MAX_LINEAR_SPEED_MPS);
    double radps = driveVel_mps / Module.WHEEL_RADIUS;
    double rps = Units.radiansToRotations(radps);
    drivePos_r += rps * LOOP_PERIOD_SECS;
  }

  @Override
  public void setTurnAngle(double r) {
    double lastTurnPos_r = turnPos_r;
    turnPos_r = r - turnAbsoluteInitPosition.getRotations();
    turnVel_rps = (turnPos_r - lastTurnPos_r) / LOOP_PERIOD_SECS;
  }
}
