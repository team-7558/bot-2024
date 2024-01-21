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

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

public class Shooter extends StateMachineSubsystemBase {

  private static double SHOOTER_HEIGHT = 0;

  private static double SPEAKER_HEIGHT_DIFFERENCE = 3; // fill
  private static Pose3d SPEAKER_POSE = new Pose3d(); // fill

  private static Shooter instance;

  public static Shooter getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new Shooter(new ShooterIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          instance = new Shooter(new ShooterIO() {});
          break;
      }
    }

    return instance;
  }

  public final State DISABLED, IDLE;

  private final ShooterIO io;

  private final SimpleMotorFeedforward ffModel;
  // hoodangle at 1 rad because angle of hood at max height is around 60 degrees, turret is 180
  // degrees turret so 90 seems like it would be ok

  /** Creates a new Flywheel. */
  private Shooter(ShooterIO io) {
    super("Shooter");
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.flywheelConfigurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.flywheelConfigurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    DISABLED =
        new State("DISABLED") {

          @Override
          public void init() {
            stop();
            // stop hood
            // stop target angle spining
          }

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

    IDLE =
        new State("IDLE") {

          @Override
          public void init() {
            stop();
          }

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

   
  }

  @Override
  public void inputPeriodic() {
    
  }

  @Override
  public void outputPeriodic() {

    // Log flywheel speed in RPM
    Logger.recordOutput("FlywheelSpeedRPM", getVelocityRPM());
  }

  /** Run closed loop at the specified velocity. */
  public void runFlywheelAtVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setFlywheelVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("FlywheelSetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return 0.0;
    //will return something different when we get the inputs

   // return Units.radiansPerSecondToRotationsPerMinute(inputs.flywheelVelocityRadPerSec);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    io.setFlywheelVoltage(volts);
  }

  /**
   * Returns the required hardware states to be able to shoot in speaker
   */
  public ShooterState getStateToSpeaker() {
    Pose2d pose2d = Drive.getInstance().getPose();
    Pose3d currentPose = new Pose3d(pose2d.getX(),SHOOTER_HEIGHT,pose2d.getY(),new Rotation3d());

    double distanceToSpeaker = Math.sqrt(Math.pow(currentPose.getX() - SPEAKER_POSE.getX(), 2) + Math.pow(currentPose.getY() - SPEAKER_POSE.getY(), 2) + Math.pow(currentPose.getZ() - SPEAKER_POSE.getZ(), 2));

    ShooterState state = new ShooterState();

    // needs to be changed if the angle to hit target at is > 0
    double angle = Math.toDegrees(Math.atan((-2 * SPEAKER_HEIGHT_DIFFERENCE)/ -distanceToSpeaker));
    // gravity squared
    double velocity = (Math.sqrt(-((96.177*(1 + Math.tan(Math.toRadians(angle)))/((2 * SPEAKER_HEIGHT_DIFFERENCE) - (2*distanceToSpeaker*Math.tan(Math.toRadians(angle))))))));
    // needs to be rotation from the thing
    double turretPosition = SPEAKER_POSE.toPose2d().getRotation().getDegrees();

    state.setHoodPosition(angle);
    state.setShooterVelocityRadPerSec(velocity);
    state.setTurretPosition(turretPosition);

    return state;
  }




  
}