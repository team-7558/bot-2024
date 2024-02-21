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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double drivePos_r = 0.0;
    public double driveVel_mps = 0.0;
    public double driveVolts_V = 0.0;
    public double[] driveCurrent_A = new double[] {};

    public Rotation2d turnAbsPos_rot2d = new Rotation2d();
    public Rotation2d turnPos_rot2d = new Rotation2d();
    public double turnVel_rps = 0.0;
    public double turnVolts_V = 0.0;
    public double[] turnCurrent_A = new double[] {};

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePos_r = new double[] {};
    public Rotation2d[] odometryTurnPos_rot2d = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocity) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Run the turn motor to the specified angle in rotations */
  public default void setTurnAngle(double pos_r) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}
