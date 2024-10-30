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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.G;
import frc.robot.OI;
import frc.robot.SS2d;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.TurretCamIO.LEDStatus;
import frc.robot.subsystems.shooter.TurretCamIO.Pipeline;
import frc.robot.util.LerpTable;
import frc.robot.util.Util;
import java.io.BufferedReader;
import java.io.FileReader;
import org.littletonrobotics.junction.Logger;

public class Shooter extends StateMachineSubsystemBase {

  private static double HEIGHT_M = 0;

  public static final double LIMELIGHT_ANGLE = Units.degreesToRadians(30);
  public static final double SPEAKER_TAG_HEIGHT = Units.inchesToMeters(56.25 + 7.0 / 8.0);
  public static final double TRAP_TAG_HEIGHT = Units.inchesToMeters(36.0 + 11.5);
  public static final double LIMELIGHT_HEIGHT = Units.inchesToMeters(11);

  public static final double TURRET_ZERO_POS = 0.2506; // 0.3;
  public static final double PIVOT_ZERO_POS = 0.01389;

  public static final double FEEDER_MIN_VEL_rps = 0, FEEDER_MAX_VEL_rps = 50;
  public static final double FLYWHEEL_MIN_VEL_rps = 0, FLYWHEEL_MAX_VEL_rps = 50;
  public static final double TURRET_MIN_POS_r = -TURRET_ZERO_POS,
      TURRET_MAX_POS_r = TURRET_ZERO_POS;
  public static final double PIVOT_MIN_POS_r = PIVOT_ZERO_POS, PIVOT_MAX_POS_r = 0.16;
  public static final double FLYWHEEL_MIN_FEED_VEL_rps = 0, FLYWHEEL_MAX_FEED_VEL_rps = 50;
  public static final double TURRET_MIN_FEED_POS_r = -Units.degreesToRotations(14),
      TURRET_MAX_FEED_POS_r = -TURRET_MIN_FEED_POS_r;
  public static final double PIVOT_MIN_FEED_POS_r = PIVOT_ZERO_POS, PIVOT_MAX_FEED_POS_r = 0.02;

  public String lastPreset = "None";

  private static LerpTable shotTimesFromDistance = new LerpTable("shottimes.lerp").compile();
  ;
  private static LerpTable shotSpeedFromDistance = new LerpTable("shotspeeds.lerp").compile();

  public static LerpTable pivotHeightFromDistance = new LerpTable("pivotheights.lerp").compile();
  public static LerpTable pivotHeightFromDistance_blue =
      new LerpTable("pivotheightsblue.lerp").compile();

  private static LerpTable turretConstraintsFromPivotPos =
      new LerpTable("turretConstraintsFromPivotPos.lerp").compile();

  private static LerpTable turretFeedConstraintsFromPivotPos =
      new LerpTable("turretFeedConstraintsFromPivotPos.lerp").compile();

  private static double TIME_TO_SHOOT = 0.2;

  private static final Pose3d SPEAKER_POSE_RED = new Pose3d(16.28, 5.53, 1.987, new Rotation3d());
  private static final Pose3d SPEAKER_POSE_BLUE = new Pose3d(0.28, 5.53, 1.987, new Rotation3d());

  private static final Translation2d SPEAKER_POSE_2D_RED = new Translation2d(16.28, 5.53);
  private static final Translation2d SPEAKER_POSE_2D_BLUE = new Translation2d(0.28, 5.53);

  private static final Pose3d TRAP_LEFT_BLUE = new Pose3d(4.641, 4.498, 1.171, new Rotation3d());
  private static final Pose3d TRAP_RIGHT_BLUE = new Pose3d(4.641, 3.713, 1.171, new Rotation3d());
  private static final Pose3d TRAP_BACK_BLUE = new Pose3d(5.321, 4.105, 1.171, new Rotation3d());
  private static final Pose3d TRAP_LEFT_RED = new Pose3d(11.905, 3.713, 1.171, new Rotation3d());
  private static final Pose3d TRAP_RIGHT_RED = new Pose3d(11.905, 4.498, 1.171, new Rotation3d());
  private static final Pose3d TRAP_BACK_RED = new Pose3d(11.220, 4.105, 1.171, new Rotation3d());

  public static class Setpoints {
    public static double DEFAULT =
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

    public boolean equals(Setpoints o) {
      return flywheel_rps == o.flywheel_rps
          && turretPos_r == o.turretPos_r
          && pivotPos_r == o.pivotPos_r;
    }

    public Setpoints copy(Setpoints o) {
      this.flywheel_rps = o.flywheel_rps;
      this.feederVel_rps = o.feederVel_rps;
      this.turretPos_r = o.turretPos_r;
      this.pivotPos_r = o.pivotPos_r;
      return this;
    }

    public Setpoints copyAndFlipTurret(Setpoints o) {
      this.flywheel_rps = o.flywheel_rps;
      this.feederVel_rps = o.feederVel_rps;
      this.turretPos_r = -o.turretPos_r;
      this.pivotPos_r = o.pivotPos_r;
      return this;
    }

    @Override
    public String toString() {
      return flywheel_rps + " | " + feederVel_rps + " | " + turretPos_r + " | " + pivotPos_r;
    }
  }

  private static Shooter instance;

  public static Shooter getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance = new Shooter(new ShooterIOTalonFx(), new TurretCamIOReal());
          break;
        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new Shooter(new ShooterIOIdeal(), new TurretCamIO() {});
          break;

        default:
          // Replayed robot, disable IO implementations
          instance = new Shooter(new ShooterIO() {}, new TurretCamIO() {});
          break;
      }
    }

    return instance;
  }

  public final State DISABLED,
      IDLE,
      HOLD,
      TRACKING,
      SHOOTING,
      BEING_FED,
      FAST_FEED,
      REV_FEED,
      ZEROING,
      MANUAL,
      SOURCE_FEEDING,
      SPITTING;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final TurretCamIO llIO;

  private final TurretCamIOInputsAutoLogged llInputs = new TurretCamIOInputsAutoLogged();

  private final ShooterIO io;

  private final Debouncer feedDebouncer = new Debouncer(0.02, DebounceType.kRising);

  public enum TargetMode {
    SPEAKER,
    TRAP,
    AMP,
    CLEAR,
    CUSTOM,
  }

  private TargetMode targetMode = TargetMode.SPEAKER;

  private Setpoints lastSetpoints, currSetpoints;

  private double manualTurretVel = 0;
  private double manualPivotVel = 0;

  private boolean ll_enabled = true;
  private boolean mws_enabled = false;

  // hoodangle at 1 rad because angle of hood at max height is around 60 degrees, turret is 180
  // degrees turret so 90 seems like it would be ok

  /** Creates a new Flywheel. */
  private Shooter(ShooterIO io, TurretCamIO llIO) {
    super("Shooter");
    System.out.println(
        "cameron is owed a steak dinner minimum of 120 usd - by Andrew Matteo Bettin (the goat)");

    this.io = io;
    this.llIO = llIO;
    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)

    // loading lerp tables

    lastSetpoints = new Setpoints(0, 0, 0.0, PIVOT_MIN_POS_r); // Default values defined here
    currSetpoints = new Setpoints().copy(lastSetpoints);

    DISABLED =
        new State("DISABLED") {

          @Override
          public void init() {
            stop();
          }

          @Override
          public void periodic() {
            stop();
          }
        };

    IDLE =
        new State("IDLE") {

          @Override
          public void init() {
            stop();
            llIO.setLEDs(LEDStatus.OFF);
          }

          @Override
          public void periodic() {
            stop();
          }

          @Override
          public void exit() {}
        };

    HOLD =
        new State("HOLD") {

          @Override
          public void init() {
            hold();
            llIO.setLEDs(LEDStatus.OFF);
          }

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

    SPITTING =
        new State("SPITTING") {
          @Override
          public void init() {
            currSetpoints.feederVel_rps = 30;
            currSetpoints.flywheel_rps = 20;
          }

          @Override
          public void periodic() {
            track();
          }
        };

    BEING_FED =
        new State("BEING_FED") {
          @Override
          public void init() {}

          @Override
          public void periodic() {
            // queueSetpoints(constrainSetpoints(shooterPipeline(), !inputs.beamBreakInActivated));
            Setpoints s = new Setpoints().copy(currSetpoints);
            if (inputs.beamBreakOutActivated) {
              s.feederVel_rps = 0;
              setCurrentState(IDLE);
            } else if (!inputs.beamBreakInActivated) {
              s.feederVel_rps = 6.7; // 7.1
            } else {
              s.feederVel_rps = 2.1; // 2.4
            }
            queueSetpoints(constrainSetpoints(s, true, false));
            track();
          }
        };

    FAST_FEED =
        new State("FAST_FEED") {
          @Override
          public void init() {}

          @Override
          public void periodic() {
            // queueSetpoints(constrainSetpoints(shooterPipeline(), !inputs.beamBreakInActivated));
            Setpoints s = new Setpoints().copy(currSetpoints);
            if (inputs.beamBreakOutActivated) {
              s.feederVel_rps = 0;
              setCurrentState(IDLE);
            } else if (!inputs.beamBreakInActivated) {
              s.feederVel_rps = 7.0; // 7.3
            } else {
              s.feederVel_rps = 2.2; // 2.3
            }
            queueSetpoints(constrainSetpoints(s, true, false));
            track();
          }
        };

    REV_FEED =
        new State("REV_FEED") {
          @Override
          public void init() {}

          @Override
          public void periodic() {
            queueSetpoints(new Setpoints(Setpoints.DEFAULT, -8.5, 0, PIVOT_MIN_POS_r));
            track();
          }
        };

    SOURCE_FEEDING =
        new State("SOURCE_FEEDING") {

          int i = G.isRedAlliance() ? -1 : 1;

          @Override
          public void init() {}

          public void periodic() {
            queueSetpoints(
                constrainSetpoints(
                    (new Setpoints(40, 40, i * 0.027, PIVOT_MIN_POS_r)), false, true));
            track();
          }
        };

    // turent locks on to target and follow it
    TRACKING =
        new State("TRACKING") {
          @Override
          public void init() {}

          @Override
          public void periodic() {
            // queueSetpoints(constrainSetpoints(shooterPipeline(), false, false));
            track();
          }

          @Override
          public void exit() {}
        };

    SHOOTING =
        new State("SHOOTING") {
          @Override
          public void init() {
            ShotLogger.log();
            llIO.snapshot();
            llIO.setLEDs(LEDStatus.OFF);
          }

          @Override
          public void periodic() {
            currSetpoints.feederVel_rps = 35;
            track();
          }

          @Override
          public void exit() {}
        };

    ZEROING =
        new State("ZEROING") {

          boolean zeroedPivot = false;

          @Override
          public void init() {
            zeroedPivot = false;
            io.setPivotVolts(-1);
          }

          @Override
          public void periodic() {

            if (true /*inputs.pivotHallEffect*/) {
              io.setPivotVolts(-.5);
              if (!zeroedPivot) {
                io.zeroPivot();
                zeroedPivot = false;
              }
              if (inputs.turretHallEffect) {
                io.setTurretVolts(0);
                io.zeroTurret();
                if (Util.inRange(inputs.turretPosR - TURRET_ZERO_POS, 0.005)
                    && Util.inRange(inputs.pivotPosR - PIVOT_ZERO_POS, 0.005)) {
                  setCurrentState(IDLE);
                }
              } else {
                if (OI.XK.get(8, 0)) {
                  io.setTurretVolts(-0.85);
                } else {
                  io.setTurretVolts(0.85);
                }
              }
            }
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
            io.setPivotVolts(manualPivotVel);
          }

          @Override
          public void exit() {
            stop();
          }
        };

    setCurrentState(DISABLED);
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    llIO.updateInputs(llInputs);
    Logger.processInputs("Shooter", inputs);
    Logger.processInputs("TurretCam", llInputs);
  }

  @Override
  public void outputPeriodic() {
    SS2d.M.setShooterTilt(inputs.pivotPosR * 360);
    SS2d.M.setTurretAngle(inputs.turretPosR * 360);
    SS2d.M.setTurretConstraints(
        turretConstraintsFromPivotPos.calcY(inputs.pivotPosR) * 360.0,
        turretFeedConstraintsFromPivotPos.calcY(inputs.pivotPosR) * 360.0);

    SS2d.S.setShooterTilt(currSetpoints.pivotPos_r * 360);
    SS2d.S.setTurretAngle(currSetpoints.turretPos_r * 360);
    SS2d.S.setTurretConstraints(
        turretConstraintsFromPivotPos.calcY(currSetpoints.pivotPos_r) * 360.0,
        turretFeedConstraintsFromPivotPos.calcY(currSetpoints.pivotPos_r) * 360.0);

    Logger.recordOutput("Shooter/TargetFeed", currSetpoints.feederVel_rps);
    Logger.recordOutput("Shooter/TargetFly", currSetpoints.flywheel_rps);
    Logger.recordOutput("Shooter/TargetTurr", currSetpoints.turretPos_r);
    Logger.recordOutput("Shooter/TargetPivot", currSetpoints.pivotPos_r);

    Logger.recordOutput("Shooter/LLonTarget", llOnTarget());
    Logger.recordOutput("Shooter/LLdist", llDist());

    if (Constants.verboseLogging) {
      Logger.recordOutput("Shooter/PrevTargetFeed", lastSetpoints.feederVel_rps);
      Logger.recordOutput("Shooter/PrevTargetFly", lastSetpoints.flywheel_rps);
      Logger.recordOutput("Shooter/PrevTargetTurr", lastSetpoints.turretPos_r);
      Logger.recordOutput("Shooter/PrevTargetPivot", lastSetpoints.pivotPos_r);
    }
  }

  /** Stops Everything */
  public void stop() {
    // queueSetpoints(new Setpoints(0, 0));
    io.stop();
  }

  public void setLLPipeline(Pipeline p) {
    llIO.setPipeline(p);
  }

  public void toggleCamera() {
    ll_enabled = !ll_enabled;
  }

  public boolean llEnabled() {
    return ll_enabled;
  }

  public void toggleMovingWhileShooting() {
    mws_enabled = !mws_enabled;
  }

  public void toggleBrake() {
    io.toggleBrake();
  }

  public void zero() {
    io.zeroTurret();
    io.zeroPivot();
  }

  public TargetMode getTargetMode() {
    return targetMode;
  }

  public static double getTargetHeight() {
    return G.isRedAlliance()
        ? Units.inchesToMeters(57.13)
        : Units.inchesToMeters(57.13); // TODO: find for blue
  }

  public void setTargetMode(TargetMode mode) {
    targetMode = mode;
  }

  private void track() {
    io.setFeederVel(currSetpoints.feederVel_rps);
    io.setFlywheelVel(currSetpoints.flywheel_rps);
    io.setTurretPos(currSetpoints.turretPos_r);
    io.setPivotPos(currSetpoints.pivotPos_r);
  }

  private void hold() {
    io.setFeederVel(0);
    io.setFlywheelVel(0);
    io.setTurretPos(inputs.turretPosR);
    io.setPivotPos(inputs.pivotPosR);
  }

  public Setpoints getMeasuredSetpoints() {
    return new Setpoints(
        inputs.flywheelVelRPS, inputs.feederVelRPS, inputs.turretPosR, inputs.pivotPosR);
  }

  public void queueSetpoints(Setpoints s) {
    // System.out.println(s);
    // Setpoints temp = new Setpoints().copy(currSetpoints);
    // currSetpoints.flywheel_rps =
    //     s.flywheel_rps == Setpoints.DEFAULT ? lastSetpoints.flywheel_rps : s.flywheel_rps;
    // currSetpoints.feederVel_rps =
    //     s.feederVel_rps == Setpoints.DEFAULT ? lastSetpoints.feederVel_rps : s.feederVel_rps;
    // currSetpoints.turretPos_r =
    //     s.turretPos_r == Setpoints.DEFAULT ? lastSetpoints.turretPos_r : s.turretPos_r;
    // currSetpoints.pivotPos_r =
    //     s.pivotPos_r == Setpoints.DEFAULT ? lastSetpoints.pivotPos_r : s.pivotPos_r;
    // lastSetpoints.copy(temp);
    currSetpoints.copy(s);
  }

  public boolean canFeed() {
    boolean res = true;

    res &= Util.inRange(currSetpoints.pivotPos_r, PIVOT_MIN_FEED_POS_r, PIVOT_MAX_FEED_POS_r);
    res &=
        Util.inRange(
            currSetpoints.turretPos_r,
            turretFeedConstraintsFromPivotPos.calcY(currSetpoints.pivotPos_r));

    return res;
  }

  public boolean isFlywheelAtSetpoint(double tol) {
    return Util.inRange(currSetpoints.flywheel_rps - inputs.flywheelVelRPS, tol);
  }

  public boolean isTurretAtSetpoint(double tol) {
    return Util.inRange(currSetpoints.turretPos_r - inputs.turretPosR, tol);
  }

  public boolean isPivotAtSetpoint(double tol) {
    return Util.inRange(currSetpoints.pivotPos_r - inputs.pivotPosR, tol);
  }

  public boolean isAtSetpoints() {
    boolean res = true;
    res &= isFlywheelAtSetpoint(2);
    res &= isTurretAtSetpoint(0.01);
    res &= isPivotAtSetpoint(0.01);
    Logger.recordOutput("Shooter/AtSetpoints", res);

    return res;
  }

  /** Returns the current velocity in RPM. */
  public boolean beamBroken() {
    return inputs.beamBreakOutActivated;
  }

  public boolean beamBrokenIn() {
    return inputs.beamBreakInActivated;
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

      double shotTime_s = TIME_TO_SHOOT;

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

    double flywheel_rps = shotSpeedFromDistance.calcY(dist);

    double theta =
        Math.IEEEremainder(Math.atan2(delta.getY(), delta.getX()) + Math.PI, 2 * Math.PI);
    double turretPos_r = Units.radiansToRotations(theta);
    double pivotPos_r = pivotHeightFromDistance.calcY(dist);

    return new Setpoints(flywheel_rps, Setpoints.DEFAULT, turretPos_r, pivotPos_r);
  }

  public Setpoints constrainSetpoints(Setpoints s, boolean isFeeding, boolean source) {
    Setpoints p = new Setpoints().copy(s);

    if (source) {
      p.flywheel_rps = Util.limit(s.flywheel_rps, FLYWHEEL_MIN_VEL_rps, FLYWHEEL_MAX_VEL_rps);
      p.pivotPos_r = Util.limit(s.pivotPos_r, PIVOT_MIN_POS_r, PIVOT_MAX_POS_r);
      double y = turretConstraintsFromPivotPos.calcY(p.pivotPos_r);
      p.turretPos_r = Util.limit(s.turretPos_r, -y, y);
      return p;
    }

    if (isFeeding) {
      p.feederVel_rps = Util.limit(s.feederVel_rps, FEEDER_MIN_VEL_rps, FEEDER_MAX_VEL_rps);
      p.flywheel_rps =
          Util.limit(s.flywheel_rps, FLYWHEEL_MIN_FEED_VEL_rps, FLYWHEEL_MAX_FEED_VEL_rps);
      p.pivotPos_r = Util.limit(s.pivotPos_r, PIVOT_MIN_FEED_POS_r, PIVOT_MAX_FEED_POS_r);
      double y = turretFeedConstraintsFromPivotPos.calcY(p.pivotPos_r);
      // System.out.println(p.pivotPos_r + ":" + y);
      p.turretPos_r = Util.limit(s.turretPos_r, -y, y);
      // p.turretPos_r = s.turretPos_r;
    } else {
      p.feederVel_rps = 0;
      p.flywheel_rps = Util.limit(s.flywheel_rps, FLYWHEEL_MIN_VEL_rps, FLYWHEEL_MAX_VEL_rps);
      p.pivotPos_r = Util.limit(s.pivotPos_r, PIVOT_MIN_POS_r, PIVOT_MAX_POS_r);
      double y = turretConstraintsFromPivotPos.calcY(p.pivotPos_r);
      p.turretPos_r = Util.limit(s.turretPos_r, -y, y);
    }

    // System.out.println(p);
    return p;
  }

  public Setpoints adjustPreset(Setpoints s) {
    double isRed = G.isRedAlliance() ? 0.5 : 0;
    double driveRot = Drive.getInstance().getRotation().getRotations();
    double rotErr = Math.IEEEremainder(isRed - driveRot, 1.0);
    Setpoints newSetpoints = new Setpoints().copy(s);
    newSetpoints.turretPos_r += rotErr;
    return newSetpoints;
  }

  Debouncer lldb = new Debouncer(0.3, DebounceType.kFalling);

  double txOffset = 0;

  public Setpoints llTakeover(Setpoints s, Pipeline p) {
    if (ll_enabled && lldb.calculate(llInputs.connected && llInputs.tv)) {
      Setpoints ns = new Setpoints().copy(s);

      if (p == Pipeline.TRAP) {
        if ((G.isRedAlliance() && (llInputs.tid == 11 || llInputs.tid == 12 || llInputs.tid == 13))
            || (!G.isRedAlliance() && (llInputs.tid > 13))) {
          double tx = llInputs.tx;
          double ty = llInputs.ty;
          double distToTarget = llDist();
          double botRot = Drive.getInstance().getRotation().getRotations();
          double aimRot = Math.IEEEremainder(botRot + inputs.turretPosR, 1.0);

          double aaaimrot = Math.IEEEremainder(G.isRedAlliance() ? aimRot + 0.5 : aimRot, 1.0);
          if (llInputs.connected && llInputs.tv) {

            double minDamp = 0.45;
            double maxDamp = 0.35;
            double minLat = 20;
            double maxLat = 200;
            ns.turretPos_r =
                inputs.turretPosR
                    - Units.degreesToRotations(tx - txOffset)
                        * Util.remap(minLat, maxLat, llInputs.latency, minDamp, maxDamp);
          } else {
            // txOffset = 0;
            double minDamp = 0.05;
            double maxDamp = 0.0;
            double minLat = 20;
            double maxLat = 200;
            ns.turretPos_r =
                inputs.turretPosR
                    - Units.degreesToRotations(llInputs.tx - txOffset)
                        * Util.remap(minLat, maxLat, llInputs.latency, minDamp, maxDamp);
          }

          Logger.recordOutput("Shooter/aimRot", aaaimrot);
          Logger.recordOutput("Shooter/txOffset", txOffset);

          // ns.flywheel_rps = shotSpeedFromDistance.calcY(distToTarget);
          // ns.pivotPos_r = pivotHeightFromDistance.calcY(distToTarget);

          if (llOnTarget()) llIO.setLEDs(LEDStatus.HI);
          else llIO.setLEDs(LEDStatus.LOW);
        }
      } else {
        if ((G.isRedAlliance() && llInputs.tid == 4) || (!G.isRedAlliance() && llInputs.tid == 7)) {

          double tx = llInputs.tx;
          double ty = llInputs.ty;
          double distToTarget = llDist();
          double mwsTx = 0;
          double botRot = Drive.getInstance().getRotation().getRotations();
          double aimRot = Math.IEEEremainder(botRot + inputs.turretPosR, 1.0);

          if (mws_enabled) {
            Translation2d turrTranslation =
                new Translation2d(distToTarget, Rotation2d.fromRotations(aimRot));
            ChassisSpeeds fieldRelChassisSpeeds = Drive.getInstance().getFieldRelativeSpeeds();

            double shotTime_s = 0.1;

            Translation2d offset =
                new Translation2d(
                    -fieldRelChassisSpeeds.vxMetersPerSecond * shotTime_s,
                    -fieldRelChassisSpeeds.vyMetersPerSecond * shotTime_s);
            Translation2d ogPos =
                G.isRedAlliance()
                    ? SPEAKER_POSE_2D_RED.plus(turrTranslation)
                    : SPEAKER_POSE_2D_BLUE.plus(turrTranslation);
            Translation2d newTarget =
                G.isRedAlliance()
                    ? SPEAKER_POSE_2D_RED.plus(offset)
                    : SPEAKER_POSE_2D_BLUE.plus(offset);
            Translation2d newTurrTranslation = newTarget.minus(ogPos);

            Logger.recordOutput("Shooter/ogPos", ogPos);
            Logger.recordOutput("Shooter/turrTranslation", turrTranslation);
            Logger.recordOutput("Shooter/newTarget", newTarget);
            Logger.recordOutput("Shooter/newTurrTranslation", newTurrTranslation);

            double newBotRot = newTurrTranslation.getAngle().getRotations();
            double newDist = newTarget.getDistance(ogPos);

            double txOffset = Units.rotationsToDegrees(botRot - newBotRot);

            tx += txOffset;
            distToTarget = newDist;

            // double botRad = Drive.getInstance().getRotation().getRadians();

            // ChassisSpeeds botSpeeds = Drive.getInstance().getFieldRelativeSpeeds();

            // double omega = botSpeeds.omegaRadiansPerSecond;

            // double angularOffsetX =
            //     Units.radiansToDegrees(
            //         Math.atan(Math.tan(llInputs.tx) * botSpeeds.vxMetersPerSecond));
            // double angularOffsetY =
            //     Units.radiansToDegrees(
            //         Math.atan(Math.tan(llInputs.ty) * botSpeeds.vyMetersPerSecond));

            // Logger.recordOutput("Shooter/xOffset", angularOffsetX);
            // Logger.recordOutput("Shooter/yOffset", angularOffsetY);

            // // tx = llInputs.tx - angularOffsetX;
            // // ty = llInputs.ty - angularOffsetY;

            // untested code it sort of makes sense logically

          }

          double aaaimrot = Math.IEEEremainder(G.isRedAlliance() ? aimRot + 0.5 : aimRot, 1.0);
          if (llInputs.connected && llInputs.tv) {
            txOffset = Util.remap(-0.25, 0.25, aaaimrot, -5, 5);

            double minDamp = 0.4;
            double maxDamp = 0.15;
            double minLat = 20;
            double maxLat = 200;
            ns.turretPos_r =
                inputs.turretPosR
                    - Units.degreesToRotations(tx - txOffset)
                        * Util.remap(minLat, maxLat, llInputs.latency, minDamp, maxDamp);
          } else {
            // txOffset = 0;
            double minDamp = 0.05;
            double maxDamp = 0.0;
            double minLat = 20;
            double maxLat = 200;
            ns.turretPos_r =
                inputs.turretPosR
                    - Units.degreesToRotations(llInputs.tx - txOffset)
                        * Util.remap(minLat, maxLat, llInputs.latency, minDamp, maxDamp);
          }

          Logger.recordOutput("Shooter/aimRot", aaaimrot);
          Logger.recordOutput("Shooter/txOffset", txOffset);

          ns.flywheel_rps = shotSpeedFromDistance.calcY(distToTarget);
          ns.pivotPos_r = pivotHeightFromDistance.calcY(distToTarget);

          if (llOnTarget()) llIO.setLEDs(LEDStatus.HI);
          else llIO.setLEDs(LEDStatus.LOW);
        }
      }

      return ns;
    } else {
      llIO.setLEDs(LEDStatus.OFF);
      return s;
    }
  }

  public boolean llHasComms() {
    return llInputs.connected;
  }

  public double lltx() {
    return llInputs.tx;
  }

  public double llty() {
    return llInputs.ty;
  }

  public boolean llHasCommsWithTarget() {
    return llInputs.connected && llInputs.tv;
  }

  public boolean llOnTarget() {
    return llInputs.connected && llInputs.tv && Math.abs(llInputs.tx - txOffset) < 2.6;
  }

  public boolean mwsEnabled() {
    return mws_enabled;
  }

  public double llDist(double ty) {
    double angleToGoal = LIMELIGHT_ANGLE + Units.degreesToRadians(ty);
    double distToTarget = (SPEAKER_TAG_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(angleToGoal);
    return distToTarget;
  }

  public double llDist() {
    double angleToGoal = LIMELIGHT_ANGLE + Units.degreesToRadians(llInputs.ty);
    double distToTarget = (SPEAKER_TAG_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(angleToGoal);
    return distToTarget;
  }

  public double llDistTrap() {
    double angleToGoal = LIMELIGHT_ANGLE + Units.degreesToRadians(llInputs.ty);
    double distToTarget = (TRAP_TAG_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(angleToGoal);
    return distToTarget;
  }

  public double getTid() {
    return llInputs.tid;
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

    if (Constants.verboseLogging) {
      Logger.recordOutput("Shooter/Target", speaker);
      Logger.recordOutput("Shooter/PositionAdjustedTarget", positionAdjustedSpeaker);
      Logger.recordOutput("Shooter/VelocityAdjustedTarget", velocityAdjustedSpeaker);
    }
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

  public void setLastPreset(String presetName) {
    this.lastPreset = presetName;
  }

  public String getLastPreset() {
    return this.lastPreset;
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

    if (Constants.verboseLogging) {
      Logger.recordOutput("Shooter/Target", trap);
      Logger.recordOutput("Shooter/PositionAdjustedTarget", trap);
      Logger.recordOutput("Shooter/VelocityAdjustedTarget", velocityAdjustedTrap);
    }
    return newSetpoints;
  }

  public void setManualTurretVel(double vel_rps) {
    manualTurretVel = vel_rps;
  }

  public void setManualPivotVel(double vel_rps) {
    manualPivotVel = vel_rps;
  }

  public Setpoints getCurrSetpoints() {
    return currSetpoints;
  }
}
