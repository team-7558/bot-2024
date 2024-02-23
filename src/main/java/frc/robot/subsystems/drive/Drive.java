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

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.drive.Module.Mode;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.Util;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateMachineSubsystemBase {
  public static final int FL = 0, FR = 1, BL = 2, BR = 3;
  public static final double MAX_LINEAR_SPEED_MPS = 4.73;
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(18.75);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(18.75);
  private static final double SKEW_CONSTANT = 0.06;
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED_RADPS = MAX_LINEAR_SPEED_MPS / DRIVE_BASE_RADIUS;

  // TODO: tune all this
  // -- VISION CONSTANTS --

  // maximum distance on high fps, low res before we switch the camera to high res lower fps
  public static final double MAX_DISTANCE = 3.0;

  // minimum distance to be on low fps high res before we switch the camera to high fps low res
  public static final double MIN_DISTANCE = 3.1;

  // distance to cut off all pose estimation because its too inaccurate
  public static final double CUTOFF_DISTANCE = 7.0;

  // ratio for the distance scaling on the standard deviation
  private static final double APRILTAG_COEFFICIENT = 0.01; // NEEDS TO BE TUNED

  public static final Lock odometryLock = new ReentrantLock();

  private static Drive instance;

  public static Drive getInstance() {

    if (instance == null) {
      System.out.println("Drive initialized");
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          // instance =
          // new Drive(
          //     new GyroIOPigeon2(),
          //     new ModuleIO2023(0),
          //     new ModuleIO2023(1),
          //     new ModuleIO2023(2),
          //     new ModuleIO2023(3));
          instance =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIO2024(0),
                  new ModuleIO2024(1),
                  new ModuleIO2024(2),
                  new ModuleIO2024(3));
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOIdeal(0),
                  new ModuleIOIdeal(1),
                  new ModuleIOIdeal(2),
                  new ModuleIOIdeal(3));
          break;

        default:
          // Replayed robot, disable IO implementations
          instance =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
          break;
      }
    }

    return instance;
  }

  public final State DISABLED, SHOOTING, PATHING, STRAFE_N_TURN, STRAFE_AUTOLOCK;

  // IO
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  public static final HolonomicPathFollowerConfig HPFG =
      new HolonomicPathFollowerConfig(
          new PIDConstants(5),
          new PIDConstants(5),
          MAX_LINEAR_SPEED_MPS,
          DRIVE_BASE_RADIUS,
          new ReplanningConfig(),
          Constants.globalDelta_sec);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private SwerveDrivePoseEstimator poseEstimator;
  private double autolockSetpoint = 0;
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

  private Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    super("Drive");
    this.gyroIO = gyroIO;
    modules[FL] = new Module(flModuleIO, FL, Mode.SETPOINT);
    modules[FR] = new Module(frModuleIO, FR, Mode.SETPOINT);
    modules[BL] = new Module(blModuleIO, BL, Mode.SETPOINT);
    modules[BR] = new Module(brModuleIO, BR, Mode.SETPOINT);

    PhoenixOdometryThread.getInstance().start();

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

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

    SHOOTING =
        new State("SHOOTING") {
          @Override
          public void periodic() {
            stopWithX();
          }
        };

    PATHING =
        new State("PATHING") {
          @Override
          public void periodic() {}
        };

    STRAFE_N_TURN =
        new State("STRAFE N TURN") {
          @Override
          public void periodic() {
            double throttle = 1.0;
            throttle = Util.lerp(1, 0.4, OI.DR.getRightTriggerAxis() * OI.DR.getRightTriggerAxis());
            drive(-OI.DR.getLeftY(), -OI.DR.getLeftX(), -OI.DR.getRightX() * 0.75, throttle);
          }
        };

    STRAFE_AUTOLOCK =
        new State("STRAFE AUTOLOCK") {
          @Override
          public void periodic() {
            double throttle = 1.0;
            throttle = Util.lerp(1, 0.4, OI.DR.getRightTriggerAxis() * OI.DR.getRightTriggerAxis());
            double err =
                Math.IEEEremainder(
                    getPose().getRotation().getRadians() - Units.degreesToRadians(autolockSetpoint),
                    Math.PI * 2.0);
            Logger.recordOutput("Drive/Autolock Heading Error", err);
            double con = Util.inRange(err, 0.35) ? 2 * err : 0.8 * err;
            Logger.recordOutput("Drive/Autolock Heading Output", con);
            drive(-OI.DR.getLeftY(), -OI.DR.getLeftX(), -con, throttle);
          }
        };
    setCurrentState(DISABLED);
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getRotation(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.05),
            VecBuilder.fill(0.5, 0.5, 0.5)); // TODO: TUNE STANDARD DEVIATIONS
  }

  @Override
  public void inputPeriodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.inputPeriodic();
    }
  }

  @Override
  public void outputPeriodic() {
    for (var module : modules) {
      module.outputPeriodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    int deltaCount =
        gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;
    for (int i = 0; i < 4; i++) {
      deltaCount = Math.min(deltaCount, modules[i].getPositionDeltas().length);
    }
    for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      var twist = kinematics.toTwist2d(wheelDeltas);
      if (gyroInputs.connected) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
        twist = new Twist2d(twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians());
        lastGyroRotation = gyroRotation;
      }
      // Apply the twist (change since last sample) to the current pose
      pose = pose.exp(twist);
    }

    // Be wary about using Timer.getFPGATimestamp in AK
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation(), getModulePositions());
    // TODO: figure out if needs to be moved into 250Hz processing loop
    chassisSpeeds = getChassisSpeedsFromModuleStates();
  }

  public void drive(double x, double y, double w, double throttle) {
    /*if (!G.isRedAlliance()) {
      x = -x;
      y = -y;
    }*/
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), Constants.driveDeadband);
    Rotation2d linearDirection = new Rotation2d(x, y);
    double omega = MathUtil.applyDeadband(w, Constants.driveDeadband);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // TODO: SKEW CORRECTION

    Logger.recordOutput("Drive/LV", linearVelocity);

    // Convert to field relative speeds & send command

    double x_ = (linearVelocity.getX() * MAX_LINEAR_SPEED_MPS) * throttle;
    double y_ = (linearVelocity.getY() * MAX_LINEAR_SPEED_MPS) * throttle;
    double w_ = omega * MAX_ANGULAR_SPEED_RADPS;

    Logger.recordOutput("Drive/xp", x_);
    Logger.recordOutput("Drive/yp", y_);
    Logger.recordOutput("Drive/wp", w_);

    ChassisSpeeds rr =
        ChassisSpeeds.fromFieldRelativeSpeeds(x_, y_, w_, getPose().getRotation() /*.plus(
                    new Rotation2d(
                        getAngularVelocity() * SKEW_CONSTANT))*/);

    runVelocity(rr); // TODO: tune skew constant
  }

  public void zeroGyro() {}

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.globalDelta_sec);
    Logger.recordOutput("Drive/CS", speeds);
    Logger.recordOutput("Drive/DCS", discreteSpeeds);

    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED_MPS);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  public void setBrakeMode(boolean brake) {
    for (var module : modules) {
      module.setBrakeMode(brake);
    }
  }

  public boolean getBrakeMode() {
    return modules[FL].getBrakeMode();
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /**
   * Returns the angular velocity of the robot
   *
   * @return radians/s
   */
  public double getAngularVelocity() {
    if (gyroInputs.connected) {
      return gyroInputs.yawVelocityRadPerSec;
    } else {
      return chassisSpeeds.omegaRadiansPerSecond;
    }
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    // TODO: figure out if this is good enough or instead use gyroInputs if it exists
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void hardSetPose(Pose2d pose) {
    this.pose = pose;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED_MPS;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED_RADPS;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      modules[FL].getPosition(),
      modules[FR].getPosition(),
      modules[BL].getPosition(),
      modules[BR].getPosition()
    };
  }

  /** Returns robot relative chassis speeds * */
  public ChassisSpeeds getChassisSpeedsFromModuleStates() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns robot relative chassis speeds * */
  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  /** Returns field relative chassis speeds * */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        getRotation());
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  /**
   * Sets the autolock setpoint
   *
   * @param degrees
   */
  public void setAutolockSetpoint(double degrees) {
    this.autolockSetpoint = degrees;
  }

  /**
   * Gets the autolock setpoint
   *
   * @return degrees
   */
  public double getAutolockSetpoint() {
    return autolockSetpoint;
  }

  public void setModuleModes(Module.Mode mode) {
    modules[0].mode = mode;
    modules[1].mode = mode;
    modules[2].mode = mode;
    modules[3].mode = mode;
  }
}
