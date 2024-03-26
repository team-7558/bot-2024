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

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.G;
import frc.robot.OI;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.drive.Module.Mode;
import frc.robot.subsystems.drive.OdometryState.VisionObservation;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Util;
import java.util.Arrays;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateMachineSubsystemBase {
  public static final int FL = 0, FR = 1, BL = 2, BR = 3;
  public static final double MAX_LINEAR_SPEED_MPS = 4.975;
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(18.75);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(18.75);
  private static final double SKEW_CONSTANT = 0.06;
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED_RADPS = MAX_LINEAR_SPEED_MPS / DRIVE_BASE_RADIUS;
  // -- VISION CONSTANTS --

  // public static final double POSE_DIFFERENCE = 1.0;

  public static final Matrix<N3, N1> odometryStdDevs = VecBuilder.fill(0.005, 0.005, 0.001);

  // maximum distance on high fps, low res before we switch the camera to high res lower fps
  public static final double MAX_DISTANCE = 2.0;

  // minimum distance to be on low fps high res before we switch the camera to high fps low res
  public static final double MIN_DISTANCE = 2.1;

  // distance to cut off all pose estimation because its too inaccurate
  public static final double CUTOFF_DISTANCE = 5.0;

  // ratio for the distance scaling on the standard deviation
  private static final double APRILTAG_COEFFICIENT = 0.05; // NEEDS TO BE TUNED

  public static final Lock odometryLock = new ReentrantLock();
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(100);

  private static Drive instance;

  public static Drive getInstance() {

    if (instance == null) {
      System.out.println("Drive initialized");
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIO2024(0),
                  new ModuleIO2024(1),
                  new ModuleIO2024(2),
                  new ModuleIO2024(3),
                  new ObjectDetectorIO() {});
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOIdeal(0),
                  new ModuleIOIdeal(1),
                  new ModuleIOIdeal(2),
                  new ModuleIOIdeal(3),
                  new ObjectDetectorIO() {});
          break;

        default:
          // Replayed robot, disable IO implementations
          instance =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ObjectDetectorIO() {});
          break;
      }
    }

    return instance;
  }

  @AutoLog
  public static class OdometryTimestampInputs {
    public double[] timestamps = new double[] {};
  }

  public final State DISABLED,
      SHOOTING,
      PATHING,
      STRAFE_N_TURN,
      STRAFE_AUTOLOCK,
      INTAKING,
      AMP_SCORING;

  // IO
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final OdometryTimestampInputsAutoLogged odometryTimestampInputs =
      new OdometryTimestampInputsAutoLogged();

  private SwerveDriveWheelPositions lastPositions = null;
  private double lastTime = 0.0;

  public static final Pose2d AMP_SCORING_POSE = getAmpPose();

  private final ObjectDetectorIO llIO;
  private final ObjectDetectorIOInputsAutoLogged llInputs = new ObjectDetectorIOInputsAutoLogged();

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  public static final HolonomicPathFollowerConfig HPFG =
      new HolonomicPathFollowerConfig(
          new PIDConstants(2.5),
          new PIDConstants(3),
          MAX_LINEAR_SPEED_MPS,
          DRIVE_BASE_RADIUS,
          new ReplanningConfig(),
          Constants.globalDelta_sec);

  public static SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  // private BetterPoseEstimator poseEstimator;
  private SwerveDrivePoseEstimator poseEstimator;
  private double autolockSetpoint_r = 0, intermediaryAutolockSetpoint_r = 0;
  private Pose2d pose;
  private Rotation2d lastGyroRotation = new Rotation2d();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

  private Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      ObjectDetectorIO llIO) {

    super("Drive");
    this.gyroIO = gyroIO;
    this.llIO = llIO;
    modules[FL] = new Module(flModuleIO, FL, Mode.SETPOINT);
    modules[FR] = new Module(frModuleIO, FR, Mode.SETPOINT);
    modules[BL] = new Module(blModuleIO, BL, Mode.SETPOINT);
    modules[BR] = new Module(brModuleIO, BR, Mode.SETPOINT);

    if (Constants.currentMode != Constants.Mode.SIM) {
      PhoenixOdometryThread.getInstance();
    }

    // TEMP Pathfinding.setPathfinder(new LocalADStarAK());
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

            double x_ = -OI.DR.getLeftY();
            double y_ = -OI.DR.getLeftX();
            double w_ = -Util.sqInput(OI.DR.getRightX());

            runVelocity(drive(x_, y_, w_ * 0.7, throttle));
          }
        };

    STRAFE_AUTOLOCK =
        new State("STRAFE AUTOLOCK") {

          double scaler = 1.0 / Math.sqrt(2);

          @Override
          public void periodic() {
            double throttle = 1.0;
            throttle = Util.lerp(1, 0.4, OI.DR.getRightTriggerAxis() * OI.DR.getRightTriggerAxis());

            double x_ = -OI.DR.getLeftY();
            double y_ = -OI.DR.getLeftX();
            double mag = Math.sqrt(x_ * x_ + y_ * y_);
            intermediaryAutolockSetpoint_r = autolockSetpoint_r;
            double err =
                Math.IEEEremainder(
                    getRotation().getRotations() - intermediaryAutolockSetpoint_r, 1.0);
            if (Constants.verboseLogging) Logger.recordOutput("Drive/Autolock Heading Error", err);
            double con = 6 * err;
            con = Util.limit(con, Util.lerp(0.7, 0.2, mag * scaler));
            if (Constants.verboseLogging) Logger.recordOutput("Drive/Autolock Heading Output", con);
            runVelocity(drive(x_, y_, -con, throttle));
          }
        };

    INTAKING =
        new State("INTAKING") {
          @Override
          public void periodic() {
            double throttle = 1.0;
            throttle = Util.lerp(1, 0.4, OI.DR.getRightTriggerAxis() * OI.DR.getRightTriggerAxis());

            double x_ = -OI.DR.getLeftY();
            double y_ = -OI.DR.getLeftX();
            double w_ = -Util.sqInput(OI.DR.getRightX());

            ChassisSpeeds rrSpeeds = drive(x_, y_, w_ * 0.5, throttle);
            if (llInputs.connected) {
              if (llInputs.tv) {
                rrSpeeds.vyMetersPerSecond += -0.01 * llInputs.tx;
                rrSpeeds.vxMetersPerSecond += 0.00 * llInputs.ty;
                rrSpeeds.omegaRadiansPerSecond += -0.005 * llInputs.tx;
              }
            }
            runVelocity(rrSpeeds);
          }
        };
    AMP_SCORING =
        new State("AMP_SCORING") {
          PPHolonomicDriveController con =
              new PPHolonomicDriveController(
                  Drive.HPFG.translationConstants,
                  Drive.HPFG.rotationConstants,
                  Constants.globalDelta_sec,
                  Drive.MAX_LINEAR_SPEED_MPS,
                  Drive.DRIVE_BASE_RADIUS);

          @Override
          public void init() {}

          @Override
          public void periodic() {
            if (closeToPose(
                Drive.AMP_SCORING_POSE, new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(1)))) {}

            PathPlannerTrajectory.State state = new PathPlannerTrajectory.State();
            state.accelerationMpsSq = 0;
            state.constraints =
                new PathConstraints(3.0, 1.5, MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS / 2);
            state.curvatureRadPerMeter = 0;
            state.velocityMps = 0;
            state.targetHolonomicRotation = AMP_SCORING_POSE.getRotation();
            state.positionMeters = AMP_SCORING_POSE.getTranslation();

            ChassisSpeeds speedsNeeded = con.calculateRobotRelativeSpeeds(getPose(), state);
            System.out.println(
                speedsNeeded.vxMetersPerSecond + " " + speedsNeeded.vyMetersPerSecond);
            runVelocity(speedsNeeded);
          }

          public void exit() {
            runVelocity(new ChassisSpeeds());
          }
        };
    setCurrentState(DISABLED);

    resetPose();
  }

  @Override
  public void inputPeriodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    odometryTimestampInputs.timestamps =
        timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    if (odometryTimestampInputs.timestamps.length == 0) {
      odometryTimestampInputs.timestamps = new double[] {Timer.getFPGATimestamp()};
    }
    timestampQueue.clear();
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();

    llIO.updateInputs(llInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    Logger.processInputs("Drive/LL", llInputs);
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

    // Calculate the min odometry position updates across all modules
    int minOdometryUpdates = 1;
    for (var module : modules) {
      if (module.getModulePositions().length < minOdometryUpdates) {
        minOdometryUpdates = 0;
        break;
      }
    }
    if (gyroInputs.connected) {
      minOdometryUpdates = Math.min(gyroInputs.odometryYawPositions.length, minOdometryUpdates);
    }
    // Pass odometry data to robot state
    for (int i = 0; i < minOdometryUpdates; i++) {
      int odometryIndex = i;
      Rotation2d yaw = gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null;
      // Get all four swerve module positions at that odometry update
      // and store in SwerveDriveWheelPositions object
      SwerveDriveWheelPositions wheelPositions =
          new SwerveDriveWheelPositions(
              Arrays.stream(modules)
                  .map(module -> module.getModulePositions()[odometryIndex])
                  .toArray(SwerveModulePosition[]::new));
      // Filtering based on delta wheel positions
      boolean includeMeasurement = true;
      if (lastPositions != null) {
        double dt = odometryTimestampInputs.timestamps[i] - lastTime;
        for (int j = 0; j < modules.length; j++) {
          double velocity =
              (wheelPositions.positions[j].distanceMeters
                      - lastPositions.positions[j].distanceMeters)
                  / dt;
          double omega = // use if we wanna max the turn setpoints
              wheelPositions.positions[j].angle.minus(lastPositions.positions[j].angle).getRadians()
                  / dt;
          // Check if delta is too large
          if (Math.abs(velocity) > MAX_LINEAR_SPEED_MPS * 5.0) {
            includeMeasurement = false;
            break;
          }
        }
      }
      // If delta isn't too large we can include the measurement.
      if (includeMeasurement) {
        lastPositions = wheelPositions;
        OdometryState.getInstance()
            .addOdometryObservation(
                new OdometryState.OdometryObservation(
                    wheelPositions, yaw, odometryTimestampInputs.timestamps[i]));
        lastTime = odometryTimestampInputs.timestamps[i];
      }
    }

    // Be wary about using Timer.getFPGATimestamp in AK

    Logger.recordOutput("Drive/Speeds", chassisSpeeds);
  }

  public ChassisSpeeds drive(double x, double y, double w, double throttle) {
    if (G.isRedAlliance()) {
      x = -x;
      y = -y;
    }
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0);
    Rotation2d linearDirection = new Rotation2d(x, y);
    double omega = MathUtil.applyDeadband(w, 0);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();

    // Convert to field relative speeds & send command

    double x_ = (linearVelocity.getX() * MAX_LINEAR_SPEED_MPS) * throttle;
    double y_ = (linearVelocity.getY() * MAX_LINEAR_SPEED_MPS) * throttle;
    double w_ = omega * MAX_ANGULAR_SPEED_RADPS;

    ChassisSpeeds rr =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x_,
            y_,
            w_,
            getPose().getRotation().plus(new Rotation2d(getAngularVelocity() * SKEW_CONSTANT)));

    return rr;
  }

  public void resetPose() {
    Pose2d p;
    if (G.isRedAlliance()) {
      p = new Pose2d(15, 5.5, Rotation2d.fromRotations(0.5));
    } else {
      p = new Pose2d(1.5, 5.5, Rotation2d.fromRotations(0));
    }
    hardSetPose(p);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.globalDelta_sec);
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

  public void toggleBrake() {
    for (var module : modules) {
      module.toggleBrake();
    }
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

  public boolean closeToPose(Pose2d reference, Pose2d tolerance) {
    Pose2d m_poseError = reference.relativeTo(getPose());
    Rotation2d m_rotationError = reference.getRotation().minus(getPose().getRotation());
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_rotationError;
    return Math.abs(eTranslate.getX()) < tolerance.getX()
        && Math.abs(eTranslate.getY()) < tolerance.getY()
        && Math.abs(eRotate.getRadians()) < tolerance.getRotation().getRadians();
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
    return OdometryState.getInstance().getEstimatedPose();
  }

  @AutoLogOutput(key = "Odometry/Estimation")
  public Pose2d getPoseEstimatorPose() {
    return OdometryState.getInstance().getEstimatedPose();
  }

  public void addToPoseEstimator(
      Pose2d pose, double timestamp, double ambiguity, double blacklistCoeff, int[] tids) {
    if (pose.getTranslation().getDistance(getPose().getTranslation()) > CUTOFF_DISTANCE) return;
    double distSums = 0;
    for (int i = 0; i < tids.length; i++) {
      try {
        Pose2d tagPose = Vision.AT_MAP.getTagPose(i).orElseThrow().toPose2d();
        distSums += tagPose.getTranslation().getDistance(getPose().getTranslation());
      } catch (Exception e) {

      }
    }
    double avgDistance = distSums / tids.length;

    OdometryState.getInstance()
        .addVisionObservation(
            new VisionObservation(
                pose,
                timestamp,
                VecBuilder.fill(
                    ((blacklistCoeff * avgDistance) - tids.length) * APRILTAG_COEFFICIENT,
                    ((blacklistCoeff * avgDistance) - tids.length) * APRILTAG_COEFFICIENT,
                    10)));
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return OdometryState.getInstance().getEstimatedPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void hardSetPose(Pose2d pose) {
    OdometryState.getInstance().resetPose(pose);
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
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeedsFromModuleStates(), getRotation());
  }

  public static Pose2d getAmpPose() {
    return G.isRedAlliance()
        ? new Pose2d(1.76, 7.71, Rotation2d.fromDegrees(90))
        : new Pose2d(1.76, 7.71, Rotation2d.fromDegrees(90));
    // TODO: fill
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
   * @param rots
   */
  public void setAutolockSetpoint(double rots) {
    this.autolockSetpoint_r = rots;
  }

  /**
   * Gets the autolock setpoint
   *
   * @return rotations
   */
  public double getAutolockSetpoint() {
    return autolockSetpoint_r;
  }

  public void setModuleModes(Module.Mode mode) {
    modules[0].mode = mode;
    modules[1].mode = mode;
    modules[2].mode = mode;
    modules[3].mode = mode;
  }
}
