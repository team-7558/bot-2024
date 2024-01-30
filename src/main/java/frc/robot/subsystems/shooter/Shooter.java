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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util;

import org.littletonrobotics.junction.Logger;

public class Shooter extends StateMachineSubsystemBase {

  private static double SHOOTER_HEIGHT = 0;

  public static double ACCELERATION = 5.0; //TODO: tune

  private static double FEEDING_ANGLE = 0.0; //TODO: tune
  private static double FEEDING_ANGLE_TOLERANCE = 0.0; //TODO: tune
  private static double FEEDING_ROTATIONS = 5; //TODO: tune

  private static double FEEDFORWARD_VOLTS = 0; //TODO: tune
  private static double GEAR_RATIO = 2; // 2:1
  private static double FLYWHEEL_MAX_RPM = 7200 * GEAR_RATIO; //TODO: tune
  private static double FLYWHEEL_SPEED = 0.75; // -1 - 1 //TODO: tune

  private static double FLYWHEEL_RADIUS = 2; // TODO: tune
  private static double FLYWHEEL_RAD_PER_SEC = (FLYWHEEL_MAX_RPM * FLYWHEEL_SPEED) * (2* Math.PI) / 60;
  private static double FLYWHEEL_RPM = FLYWHEEL_MAX_RPM * FLYWHEEL_SPEED; // remove if unused

  private static double TURRET_SNAP_TOLERANCE = 50;

  private static Pose3d SPEAKER_POSE = new Pose3d(); // fill
  private static double HEIGHT_DIFFERENCE = SPEAKER_POSE.getZ() - SHOOTER_HEIGHT;

  private double feederPosition = 0;
  private double shooterVelocity = 0;
  private double shooterAngle = 0;
  private double turretAngle = 0;

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

  public final State DISABLED, IDLE, LOCKONT, SHOOTING,BEING_FED;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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

    
    BEING_FED = new State("BEING_FED") {
      @Override
      public void init() {
        turretAngle = FEEDING_ANGLE;
      }

      @Override
      public void periodic() {
        if(inputs.beamBreakActivated) {
          io.stopFeeder();
        }
      }
      
    };

    // turent locks on to target and follow it
    LOCKONT =
        new State("LOCKONT") {
          @Override
          public void init() {
          }

          @Override
          public void periodic() {
            ShooterState shooterState = getStateToSpeaker();
            turretAngle = shooterState.getTurretPosition();
            shooterAngle = shooterState.getHoodPosition();
            shooterVelocity = shooterState.getShooterVelocity();
          }

          @Override
          public void exit() {

          }
        };

    SHOOTING =
        new State("SHOOTING") {
          @Override
          public void init() {
            feederPosition = inputs.feederPosition + FEEDING_ROTATIONS;
          }

          @Override
          public void periodic() {
            ShooterState shooterState = getStateToSpeaker();
            shooterVelocity = shooterState.getShooterVelocity();
            turretAngle = shooterState.getTurretPosition();
            shooterAngle = shooterState.getHoodPosition();
          }


          @Override
          public void exit() {}
        };
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
  }

  @Override
  public void outputPeriodic() {

    // Log flywheel speed in RPM
    Logger.recordOutput("FlywheelSpeedRPM", getVelocityRPM());

    io.setAngle(shooterAngle);
    if((turretAngle - inputs.turretPositionDeg) > TURRET_SNAP_TOLERANCE) {
      io.setTurretAngleMotionProfile(turretAngle);
    } else {
      io.setTurretAngle(turretAngle);
    }
    io.setFeederPosition(feederPosition);
    io.setFlywheelVelocity(shooterVelocity, FEEDFORWARD_VOLTS);
  }

  /** Run closed loop at the specified velocity. */
  public void runFlywheelAtVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setFlywheelVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("FlywheelSetpointRPM", velocityRPM);
  }

  /** Stops Everything */
  public void stop() {
    shooterAngle = 0;
    turretAngle = 0;
    shooterVelocity = 0;
    io.stopFeeder();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    
    

    return Units.radiansPerSecondToRotationsPerMinute(inputs.flywheelVelocityRadPerSec);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    io.setFlywheelVoltage(volts);
  }

  /** Returns the required hardware states to be able to shoot in speaker */
  public ShooterState getStateToSpeaker() {

    // check if moving

    Drive drive = Drive.getInstance();

    ChassisSpeeds speeds = drive.getChassisSpeeds();
    Pose2d pose2d = Drive.getInstance().getPose();
 


    double distanceToSpeaker = pose2d.getTranslation().getDistance(SPEAKER_POSE.toPose2d().getTranslation());

    ShooterState state = new ShooterState();

    // TODO: MAKE ACTUALLY WORK

    if(speeds.vyMetersPerSecond > 0 || speeds.vxMetersPerSecond > 0 && speeds.omegaRadiansPerSecond == 0) {
      double[] velocityVector = new double[]{speeds.vxMetersPerSecond,speeds.vyMetersPerSecond};
      Transform3d difference = new Pose3d(pose2d).minus(SPEAKER_POSE);
      double[] speakerDifferenceVector = new double[]{difference.getX(),difference.getY(),difference.getZ()};

      double[] shoot_vector = new double[]{speakerDifferenceVector[0] - velocityVector[0], speakerDifferenceVector[1] - velocityVector[1]};

      double shoot_heading = Math.atan(shoot_vector[1] / shoot_vector[0]); // might not work 
      double shoot_speed = Math.sqrt(Math.pow(shoot_vector[0],2) + Math.pow(shoot_vector[1],2));

      double shoot_speed_rad_per_sec = shoot_speed * FLYWHEEL_RADIUS;
      double launch_angle = Math.atan((HEIGHT_DIFFERENCE) / distanceToSpeaker);
      state.setHoodPosition(launch_angle);
      state.setShooterVelocityRadPerSec(shoot_speed_rad_per_sec);
      state.setTurretPosition(shoot_heading);
      return state;
    } 



    // needs to be changed if the angle to hit target at is > 0
    double angle = Math.atan((HEIGHT_DIFFERENCE) / distanceToSpeaker);

    // needs to be rotation from the thing
    double turretPosition = Math.atan((SPEAKER_POSE.getY() - pose2d.getY()) / (SPEAKER_POSE.getX() - pose2d.getX()));

    state.setHoodPosition(angle);
    state.setShooterVelocityRadPerSec(FLYWHEEL_RAD_PER_SEC);
    state.setTurretPosition(turretPosition);
    return state;
  }

  


}
