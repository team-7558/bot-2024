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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Module {
  public static final double FUDGE_FACTOR = 1.0;
  public static final double WHEEL_RADIUS = FUDGE_FACTOR * Units.inchesToMeters(2.0);
  public static final double RPS_TO_MPS = WHEEL_RADIUS * Math.PI * 2.0;
  public static final double ODOMETRY_FREQUENCY = 250.0;

  public enum Mode {
    VOLTAGE,
    SETPOINT,
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final SimpleMotorFeedforward driveFeedforward;
  private final PIDController driveFeedback;
  private final PIDController turnFeedback;
  public Mode mode = Mode.VOLTAGE;
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private double lastPositionMeters = 0.0; // Used for delta calculation
  private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {};

  private boolean isBrake;

  public Module(ModuleIO io, int index, Mode mode) {
    this.io = io;
    this.index = index;
    this.mode = mode;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        driveFeedback = new PIDController(0.05, 0.0, 0.0);
        turnFeedback = new PIDController(7.0, 0.0, 0.0);
        break;
      case SIM:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
        driveFeedback = new PIDController(0.1, 0.0, 0.0);
        turnFeedback = new PIDController(10.0, 0.0, 0.0);
        break;
      default:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        driveFeedback = new PIDController(0.0, 0.0, 0.0);
        turnFeedback = new PIDController(0.0, 0.0, 0.0);
        break;
    }

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void inputPeriodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
  }

  public void outputPeriodic() {
    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet

    if (mode == Mode.VOLTAGE) {
      // Run closed loop turn control
      if (angleSetpoint != null) {
        io.setTurnVoltage(
            turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

        // Run closed loop drive control
        // Only allowed if closed loop turn control is running
        if (speedSetpoint != null) {
          // Scale velocity based on turn error
          //
          // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
          // towards the setpoint, its velocity should increase. This is achieved by
          // taking the component of the velocity in the direction of the setpoint.
          double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

          // Run drive controller
          double velocityRotPerSec = Units.radiansToRotations(adjustSpeedSetpoint / WHEEL_RADIUS);
          io.setDriveVoltage(
              driveFeedforward.calculate(velocityRotPerSec)
                  + driveFeedback.calculate(inputs.driveVel_mps, velocityRotPerSec));
        }
      }
    } else {
      // Run closed loop turn control
      if (angleSetpoint != null) {
        io.setTurnAngle(angleSetpoint.getRotations());

        // io.setTurnVoltage(
        //    turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

        // Run closed loop drive control
        // Only allowed if closed loop turn control is running
        if (speedSetpoint != null) {
          // Scale velocity based on turn error
          //
          // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
          // towards the setpoint, its velocity should increase. This is achieved by
          // taking the component of the velocity in the direction of the setpoint.
          double adjustSpeedSetpoint =
              speedSetpoint
                  * Math.cos(angleSetpoint.getRadians() - inputs.turnPos_rot2d.getRadians());

          // Run drive controller
          io.setDriveVelocity(adjustSpeedSetpoint);
        }
      }
    }

    // Calculate position deltas for odometry
    int deltaCount =
        Math.min(inputs.odometryDrivePos_r.length, inputs.odometryTurnPos_rot2d.length);
    positionDeltas = new SwerveModulePosition[deltaCount];
    for (int i = 0; i < deltaCount; i++) {
      double positionMeters = Units.rotationsToRadians(inputs.odometryDrivePos_r[i]) * WHEEL_RADIUS;
      Rotation2d angle = inputs.odometryTurnPos_rot2d[i];
      positionDeltas[i] = new SwerveModulePosition(positionMeters - lastPositionMeters, angle);
      lastPositionMeters = positionMeters;
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void toggleBrake() {
    io.toggleBrake();
  }

  public boolean getBrakeMode() {
    return isBrake;
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPos_rot2d;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return Units.rotationsToRadians(inputs.drivePos_r) * WHEEL_RADIUS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVel_mps;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position deltas received this cycle. */
  public SwerveModulePosition[] getPositionDeltas() {
    return positionDeltas;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return Units.rotationsToRadians(inputs.driveVel_mps);
  }

  public SwerveModulePosition[] getModulePositions() {
    int minOdometryPositions =
        Math.min(inputs.odometryDrivePos_r.length, inputs.odometryTurnPos_rot2d.length);
    SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
    for (int i = 0; i < minOdometryPositions; i++) {
      positions[i] =
          new SwerveModulePosition(
              inputs.odometryDrivePos_r[i] * RPS_TO_MPS, inputs.odometryTurnPos_rot2d[i]);
    }
    return positions;
  }
}
