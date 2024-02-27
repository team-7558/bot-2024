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

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] currentAmps = new double[] {};

    public double directionVelocityRadPerSec = 0.0;
    public double directionAppliedVolts = 0.0;

    public boolean beamBreakActivated = false;
  }

  int intakeVelocityRadPerSec = 0;

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setIntakeVoltage(double volts) {}

  /** Run the drive motor at the specified voltage. */
  public default void setIntakeSpeed(double speed) {}

  /** Run the drive motor at the specified velocity. */
  public default void setIntakeVelocity(double velocity) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDirectionVoltage(double volts) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDirectionSpeed(double speed) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDirectionVelocity(double velocity) {}

  public default void stop() {}

  public default void toggleBrake() {}
}
