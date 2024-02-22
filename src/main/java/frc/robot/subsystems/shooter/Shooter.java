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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.SS2d;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LinearInterpolator;
import frc.robot.util.Util;
import java.io.BufferedReader;
import java.io.FileReader;
import org.littletonrobotics.junction.Logger;

public class Shooter extends StateMachineSubsystemBase {

  private static double HEIGHT_M = 0;

  public static final double FLYWHEEL_MIN_VEL_rps = 0, FLYWHEEL_MAX_VEL_rps = 100;
  public static final double TURRET_MIN_POS_r = -0.25, TURRET_MAX_POS_r = 0.25;
  public static final double PIVOT_MIN_POS_r = 0, PIVOT_MAX_POS_r = 0.2;
  public static final double FLYWHEEL_MIN_FEED_VEL_rps = 0, FLYWHEEL_MAX_FEED_VEL_rps = 100;
  public static final double TURRET_MIN_FEED_POS_r = -0.02, TURRET_MAX_FEED_POS_r = 0.02;
  public static final double PIVOT_MIN_FEED_POS_r = 0, PIVOT_MAX_FEED_POS_r = 0.1;

  private static LinearInterpolator shotTimesFromDistance =
      new LinearInterpolator(getLerpTableFromFile("shottimes.lerp"));
  ;
  private static LinearInterpolator shotSpeedFromDistance =
      new LinearInterpolator(getLerpTableFromFile("shotspeeds.lerp"));

  private static LinearInterpolator turretConstraintsFromPivotPos = 
      new LinearInterpolator(getLerpTableFromFile("turretConstraintFromPivotPos.lerp"));

  private static double TIME_TO_SHOOT = 0.25;

  private static final Pose3d SPEAKER_POSE_RED = new Pose3d(16.28, 5.53, 1.987, new Rotation3d());
  private static final Pose3d SPEAKER_POSE_BLUE = new Pose3d(0.28, 5.53, 1.987, new Rotation3d());

  private static final Pose3d TRAP_LEFT_BLUE = new Pose3d(4.641, 4.498, 1.171, new Rotation3d());
  private static final Pose3d TRAP_RIGHT_BLUE = new Pose3d(4.641, 3.713, 1.171, new Rotation3d());
  private static final Pose3d TRAP_BACK_BLUE = new Pose3d(5.321, 4.105, 1.171, new Rotation3d());
  private static final Pose3d TRAP_LEFT_RED = new Pose3d(11.905, 3.713, 1.171, new Rotation3d());
  private static final Pose3d TRAP_RIGHT_RED = new Pose3d(11.905, 4.498, 1.171, new Rotation3d());
  private static final Pose3d TRAP_BACK_RED = new Pose3d(11.220, 4.105, 1.171, new Rotation3d());

  public static class Setpoints {
    private static double DEFAULT =
        -7558.0; // This way we can handle the behaviour of the setpoints specifically

    public double flywheel_rps, feederVel_rps, turretPos_r, pivotPos_r;

    public Setpoints(
        double flywheel_rps, double feederVel_rps, double turretPos_r, double pivotPos_r) {
      this.flywheel_rps = flywheel_rps;
      this.feederVel_rps = feederVel_rps;
      this.pivotPos_r = pivotPos_r;
      this.turretPos_r = turretPos_r;
    }

    public Setpoints(double flywheel_rps, double turretPos_r, double pivotPos_r) {
      this(flywheel_rps, DEFAULT, turretPos_r, pivotPos_r);
    }

    public Setpoints(double flywheel_rps, double feederVel_rps) {
      this(flywheel_rps, feederVel_rps, DEFAULT, DEFAULT);
    }

    public Setpoints(double feederVel_rps) {
      this(DEFAULT, feederVel_rps, DEFAULT, DEFAULT);
    }

    public Setpoints() {
      this(DEFAULT, DEFAULT);
    }

    public Setpoints copy(Setpoints o) {
      this.flywheel_rps = o.flywheel_rps;
      this.feederVel_rps = o.feederVel_rps;
      this.turretPos_r = o.turretPos_r;
      this.pivotPos_r = o.pivotPos_r;
      return this;
    }
  }

  private static Shooter instance;

  public static Shooter getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance = new Shooter(new ShooterIOTalonFx());
          break;
        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new Shooter(new ShooterIOIdeal());
          break;

        default:
          // Replayed robot, disable IO implementations
          instance = new Shooter(new ShooterIO() {});
          break;
      }
    }

    return instance;
  }

  public final State DISABLED, IDLE, TRACKING, SHOOTING, BEING_FED, MANUAL;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final ShooterIO io;

  public enum TargetMode {
    SPEAKER,
    TRAP,
    CUSTOM
  }

  private TargetMode targetMode = TargetMode.SPEAKER;

  private Setpoints lastSetpoints, currSetpoints;

  // hoodangle at 1 rad because angle of hood at max height is around 60 degrees, turret is 180
  // degrees turret so 90 seems like it would be ok

  /** Creates a new Flywheel. */
  private Shooter(ShooterIO io) {
    super("Shooter");

    this.io = io;
    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)

    // loading lerp tables

    lastSetpoints = new Setpoints(0, 0, 0.0, 0.0); // Default values defined here
    currSetpoints = new Setpoints().copy(lastSetpoints);

    DISABLED =
        new State("DISABLED") {

          @Override
          public void init() {
            stop();
          }
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

    BEING_FED =
        new State("BEING_FED") {
          @Override
          public void init() {}

          @Override
          public void periodic() {
            queueSetpoints(constrainSetpoints(shooterPipeline(), !inputs.beamBreakActivated));
          }
        };

    // turent locks on to target and follow it
    TRACKING =
        new State("TRACKING") {
          @Override
          public void init() {}

          @Override
          public void periodic() {
            //queueSetpoints(constrainSetpoints(shooterPipeline(), false));
            track();
          }

          @Override
          public void exit() {}
        };

    SHOOTING =
        new State("SHOOTING") {
          @Override
          public void init() {}

          @Override
          public void periodic() {
            queueSetpoints(new Setpoints(0.2));
            track();
            // feederVel_rps = 1;
            // output.setFlywheelVel(1);
          }

          @Override
          public void exit() {}
        };

    MANUAL =
        new State("MANUAL") {
          @Override
          public void init() {
            stop();
          }

          @Override
          public void periodic() {
            io.setFeederVolts(OI.DR.getLeftTriggerAxis() * 0.4 * 12);
            io.setFlywheelVolts(OI.DR.getRightTriggerAxis() * 0.6 * 12);
          }

          @Override
          public void exit() {
            stop();
          }
        };

    setCurrentState(DISABLED);

    Timer.delay(3); // TODO: periodically set this in disabled
    io.zero();
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public void outputPeriodic() {
    SS2d.M.setShooterTilt(inputs.pivotPosR * 360);
    SS2d.M.setTurretAngle(inputs.turretPosR * 360);

    SS2d.S.setShooterTilt(currSetpoints.pivotPos_r * 360);
    SS2d.S.setTurretAngle(currSetpoints.turretPos_r * 360);

    // Log flywheel speed in RPM
    Logger.recordOutput("Shooter/FlywheelSpeedRPM", getVelocityRPM());

    Logger.recordOutput("Shooter/TargetFeed", currSetpoints.feederVel_rps);
    Logger.recordOutput("Shooter/TargetFly", currSetpoints.flywheel_rps);
    Logger.recordOutput("Shooter/TargetTurr", currSetpoints.turretPos_r);
    Logger.recordOutput("Shooter/TargetPivot", currSetpoints.pivotPos_r);
  }

  /** Stops Everything */
  public void stop() {
    queueSetpoints(new Setpoints(0, 0));
    io.stop();
  }

  public void setTargetMode(TargetMode mode) {
    targetMode = mode;
  }

  private void track(){
    io.setFeederVel(currSetpoints.feederVel_rps);
    io.setFlywheelVel(currSetpoints.flywheel_rps);
    io.setTurretPos(currSetpoints.turretPos_r);
    io.setPivotPos(currSetpoints.pivotPos_r);
  }

  public void queueSetpoints(Setpoints s) {
    Setpoints temp = new Setpoints().copy(currSetpoints);
    currSetpoints.flywheel_rps =
        s.flywheel_rps == Setpoints.DEFAULT ? lastSetpoints.flywheel_rps : s.flywheel_rps;
    currSetpoints.feederVel_rps =
        s.feederVel_rps == Setpoints.DEFAULT ? lastSetpoints.feederVel_rps : s.feederVel_rps;
    currSetpoints.turretPos_r =
        s.turretPos_r == Setpoints.DEFAULT ? lastSetpoints.turretPos_r : s.turretPos_r;
    currSetpoints.pivotPos_r =
        s.pivotPos_r == Setpoints.DEFAULT ? lastSetpoints.pivotPos_r : s.pivotPos_r;
    lastSetpoints.copy(temp);
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {

    return 60 * (inputs.flywheelVelRPS);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    io.setFlywheelVolts(volts);
  }

  /**
   * Gives pose for closest trap adjusted for alliance
   *
   * @return
   */
  private static Pose3d getClosestTrap(Pose3d botpose) {
    boolean red =
        !DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Red;
    if (red) {
      double l =
          Util.dist2(botpose.getX() - TRAP_LEFT_RED.getX(), botpose.getY() - TRAP_LEFT_RED.getY());
      double r =
          Util.dist2(
              botpose.getX() - TRAP_RIGHT_RED.getX(), botpose.getY() - TRAP_RIGHT_RED.getY());
      double b =
          Util.dist2(botpose.getX() - TRAP_BACK_RED.getX(), botpose.getY() - TRAP_BACK_RED.getY());

      if (l < r) {
        if (l < b) return TRAP_LEFT_RED;
        else return TRAP_BACK_RED;
      } else {
        if (r < b) return TRAP_RIGHT_RED;
        else return TRAP_BACK_RED;
      }

    } else {
      double l =
          Util.dist2(
              botpose.getX() - TRAP_LEFT_BLUE.getX(), botpose.getY() - TRAP_LEFT_BLUE.getY());
      double r =
          Util.dist2(
              botpose.getX() - TRAP_RIGHT_BLUE.getX(), botpose.getY() - TRAP_RIGHT_BLUE.getY());
      double b =
          Util.dist2(
              botpose.getX() - TRAP_BACK_BLUE.getX(), botpose.getY() - TRAP_BACK_BLUE.getY());

      if (l < r) {
        if (l < b) return TRAP_LEFT_BLUE;
        else return TRAP_BACK_BLUE;
      } else {
        if (r < b) return TRAP_RIGHT_BLUE;
        else return TRAP_BACK_BLUE;
      }
    }
  }

  /**
   * Gives pose for speaker adjusted for alliance
   *
   * @return
   */
  private static Pose3d getSpeakerPose() {
    return !DriverStation.getAlliance().isEmpty()
            && DriverStation.getAlliance().get() == Alliance.Red
        ? SPEAKER_POSE_RED
        : SPEAKER_POSE_BLUE;
  }

  /**
   * Moves the pose of the speaker slightly based on vector to target
   *
   * @param botpose 3D botpose where height is shooter pivot height
   * @param speakerPose3d 3D pose of speaker where height is bottom of speaker hole, xy is center of
   *     speaker
   * @return
   */
  private Pose3d transformSpeakerPoseFromBotpose(Pose3d botpose, Pose3d speakerPose3d) {
    // TODO: do later
    return speakerPose3d;
  }

  /**
   * The move and shoot code, returns the moved target to aim at
   *
   * @param botpose 3D botpose where height is shooter pivot height
   * @param field_relSpeeds Field relative velocity of bot
   * @param target 3D pose of target to shoot at
   * @return
   */
  private Pose3d transformTargetPoseFromFieldRelativeVelocity(
      Pose3d botpose, ChassisSpeeds field_relSpeeds, Pose3d target) {
    Pose3d newTarget = target;
    int iterations = 1;
    for (int i = 0; i < iterations; i++) {
      Transform3d delta = newTarget.minus(botpose);
      double dist2 =
          delta.getX() * delta.getX() + delta.getY() * delta.getY() + delta.getZ() * delta.getZ();
      double dist = Math.sqrt(dist2);

      double shotTime_s = TIME_TO_SHOOT + shotTimesFromDistance.getInterpolatedValue(dist);

      Transform3d offset =
          new Transform3d(
              -field_relSpeeds.vxMetersPerSecond * shotTime_s,
              -field_relSpeeds.vyMetersPerSecond * shotTime_s,
              0.0,
              new Rotation3d());
      newTarget = target.plus(offset);
    }

    return newTarget;
  }

  /**
   * @param botpose 3D botpose where height is shooter pivot height
   * @param target 3D pose of target to shoot at
   * @return
   */
  private Setpoints calculateSetpointsForPose(Pose3d botpose, Pose3d target) {
    Transform3d delta = target.minus(botpose);
    double dist2 =
        delta.getX() * delta.getX() + delta.getY() * delta.getY() + delta.getZ() * delta.getZ();
    double dist = Math.sqrt(dist2);

    double flywheel_rps = shotSpeedFromDistance.getInterpolatedValue(dist);

    double theta =
        Math.IEEEremainder(Math.atan2(delta.getY(), delta.getX()) + Math.PI, 2 * Math.PI);
    double turretPos_r = Units.radiansToRotations(theta);
    double pivotPos_r = Units.radiansToRotations(Math.asin(delta.getZ() / dist));

    return new Setpoints(flywheel_rps, turretPos_r, pivotPos_r);
  }

  private Setpoints constrainSetpoints(Setpoints s, boolean isFeeding) { // TODO: constrain
    if (isFeeding) {
      s.feederVel_rps = 0.25;
      s.flywheel_rps = Util.limit(s.flywheel_rps, FLYWHEEL_MIN_FEED_VEL_rps, FLYWHEEL_MAX_FEED_VEL_rps);
      s.pivotPos_r = Util.limit(s.pivotPos_r, PIVOT_MIN_FEED_POS_r, PIVOT_MAX_FEED_POS_r);
      s.turretPos_r = Util.limit(s.turretPos_r, TURRET_MIN_FEED_POS_r, TURRET_MAX_FEED_POS_r);
    } else {
      s.feederVel_rps = 0;
      s.flywheel_rps = Util.limit(s.flywheel_rps, FLYWHEEL_MIN_VEL_rps, FLYWHEEL_MAX_VEL_rps);
      s.pivotPos_r = Util.limit(s.pivotPos_r, PIVOT_MIN_POS_r, PIVOT_MAX_POS_r);
      s.turretPos_r = Util.limit(s.turretPos_r, turretConstraintsFromPivotPos.getInterpolatedValue(s.pivotPos_r));
    }
    return s;
  }

  public Setpoints shooterPipeline() {
    switch (targetMode) {
      case SPEAKER:
        return speakerPipeline();
      case TRAP:
        return trapPipeline();
      case CUSTOM:
        return new Setpoints();
      default:
        System.out.println("Umm");
        return new Setpoints();
    }
  }

  public Setpoints speakerPipeline() {
    Pose2d botpose2d = Drive.getInstance().getPose();
    Pose3d botpose =
        new Pose3d(
            botpose2d.getX(),
            botpose2d.getY(),
            HEIGHT_M,
            new Rotation3d(0, 0, botpose2d.getRotation().getRadians()));
    ChassisSpeeds fieldRelSpeeds = Drive.getInstance().getFieldRelativeSpeeds();

    Pose3d speaker = getSpeakerPose();
    Pose3d positionAdjustedSpeaker = transformSpeakerPoseFromBotpose(botpose, speaker);
    Pose3d velocityAdjustedSpeaker =
        transformTargetPoseFromFieldRelativeVelocity(
            botpose, fieldRelSpeeds, positionAdjustedSpeaker);
    Setpoints newSetpoints = calculateSetpointsForPose(botpose, velocityAdjustedSpeaker);

    Logger.recordOutput("Shooter/Target", speaker);
    Logger.recordOutput("Shooter/PositionAdjustedTarget", positionAdjustedSpeaker);
    Logger.recordOutput("Shooter/VelocityAdjustedTarget", velocityAdjustedSpeaker);
    return newSetpoints;
  }

  private static double[][] getLerpTableFromFile(String name) {
    try (BufferedReader reader =
        new BufferedReader(new FileReader(Filesystem.getDeployDirectory().getPath() + name))) {
      String[] lines = reader.lines().toArray(String[]::new);
      double[][] lerp_array = new double[lines.length][2];
      for (int i = 0; i < lines.length; i++) {
        String[] xy = lines[i].split(" ");
        lerp_array[i] = new double[] {Double.parseDouble(xy[0]), Double.parseDouble(xy[1])};
      }
      return lerp_array;
    } catch (Exception e) {
      // return a default array so the whole code doesn't crash
      return new double[][] {{0, 0}};
    }
  }

  public Setpoints trapPipeline() {
    Pose2d botpose2d = Drive.getInstance().getPose();
    Pose3d botpose =
        new Pose3d(
            botpose2d.getX(),
            botpose2d.getY(),
            HEIGHT_M,
            new Rotation3d(0, 0, botpose2d.getRotation().getRadians()));
    ChassisSpeeds fieldRelSpeeds = Drive.getInstance().getFieldRelativeSpeeds();

    Pose3d trap = getClosestTrap(botpose);
    Pose3d velocityAdjustedTrap =
        transformTargetPoseFromFieldRelativeVelocity(botpose, fieldRelSpeeds, trap);
    Setpoints newSetpoints = calculateSetpointsForPose(botpose, velocityAdjustedTrap);

    Logger.recordOutput("Shooter/Target", trap);
    Logger.recordOutput("Shooter/PositionAdjustedTarget", trap);
    Logger.recordOutput("Shooter/VelocityAdjustedTarget", velocityAdjustedTrap);
    return newSetpoints;
  }
}
