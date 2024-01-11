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

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Ideal implementation of module IO.
 *
 * <p> Tracks any setpoint without any amount of error, acts as an "ideal" module
 */
public class ModuleIOIdeal implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = Constants.globalDelta_sec;

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);

  private double drivePos_rad = 0.0;  
  private double driveVel_radps = 0.0;  
  private double driveVolts_V = 0.0;

  private double turnPos_rad = 0.0;  
  private double turnVel_radps = 0.0;  
  private double turnVolts_V = 0.0;



  private int index;

  public ModuleIOIdeal(int index) {
    this.index = index;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = drivePos_rad;
    inputs.driveVelocityRadPerSec = driveVel_radps;
    inputs.driveAppliedVolts = driveVolts_V;
    inputs.driveCurrentAmps = new double[] {0};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnPos_rad).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnPos_rad);
    inputs.turnVelocityRadPerSec = turnVel_radps;
    inputs.turnAppliedVolts = turnVolts_V;
    inputs.turnCurrentAmps = new double[] {Math.abs(0)};

    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveVolts_V = MathUtil.clamp(volts, -12.0, 12.0);
    setDriveVelocity((driveVolts_V/12.0) * Drive.MAX_LINEAR_SPEED);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnVolts_V = MathUtil.clamp(volts, -12.0, 12.0);
    turnVel_radps = (turnVolts_V/12.0) * Drive.MAX_ANGULAR_SPEED;
    turnPos_rad += turnVel_radps * LOOP_PERIOD_SECS;
  }

  @Override
  public void setDriveVelocity(double velocity) {
    driveVel_radps = MathUtil.clamp(velocity, -Drive.MAX_LINEAR_SPEED, Drive.MAX_LINEAR_SPEED) / Module.WHEEL_RADIUS;
    drivePos_rad += driveVel_radps * LOOP_PERIOD_SECS;
  }

  @Override
  public void setTurnAngle(double rad) {
    double lastTurnPos_rad = turnPos_rad;
    turnPos_rad = rad;
    turnVel_radps = (turnPos_rad - lastTurnPos_rad) / LOOP_PERIOD_SECS;
  }
}
