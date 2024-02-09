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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LinearInterpolator;
import org.littletonrobotics.junction.Logger;

public class Shooter extends StateMachineSubsystemBase {

  private static double SHOOTER_HEIGHT = 0;

  public static double ACCELERATION = 5.0; // TODO: tune

  private static double FEEDING_ANGLE = 0.0; // TODO: tune
  private static double FEEDING_ANGLE_TOLERANCE = 0.0; // TODO: tune
  private static double FEEDING_ROTATIONS = 5; // TODO: tune

  private static double FEEDFORWARD_VOLTS = 0; // TODO: tune
  private static double GEAR_RATIO = 2; // 2:1
  private static double FLYWHEEL_MAX_RPM = 7200 * GEAR_RATIO; // TODO: tune
  private static double FLYWHEEL_SPEED = 0.75; // -1 - 1 //TODO: tune

  private static double FLYWHEEL_RADIUS = 2; // TODO: tune
  private static double FLYWHEEL_RAD_PER_SEC =
      (FLYWHEEL_MAX_RPM * FLYWHEEL_SPEED) * (2 * Math.PI) / 60;
  private static double FLYWHEEL_RPM = FLYWHEEL_MAX_RPM * FLYWHEEL_SPEED; // remove if unused

  private static final double[][] TABLE_DATA = new double[][] {{0, 0}, {0, 1}}; // TODO: fill
  private static final LinearInterpolator DISTANCE_TABLE = new LinearInterpolator(TABLE_DATA);

  private static double TIME_TO_SHOOT = 0.5;

  private static double TURRET_SNAP_TOLERANCE = 50;

  private static Pose3d SPEAKER_POSE = getSpeakerPose();
  private static double HEIGHT_DIFFERENCE = SPEAKER_POSE.getZ() - SHOOTER_HEIGHT;

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

  public final State DISABLED, IDLE, LOCKONT, SHOOTING, BEING_FED;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final ShooterIO io;

  private Setpoints lastSetpoints, currSetpoints;

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
        io.flywheelConfigurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        io.flywheelConfigurePID(0.5, 0.0, 0.0);
        break;
      default:
        break;
    }

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
            if (inputs.beamBreakActivated) {
              lastSetpoints.feederVel_rps = currSetpoints.feederVel_rps;
              currSetpoints.feederVel_rps = 0;
            }
          }
        };

    // turent locks on to target and follow it
    LOCKONT =
        new State("LOCKONT") {
          @Override
          public void init() {}

          @Override
          public void periodic() {}

          @Override
          public void exit() {}
        };

    SHOOTING =
        new State("SHOOTING") {
          @Override
          public void init() {}

          @Override
          public void periodic() {
            // feederVel_rps = 1;
            // output.setFlywheelVel(1);
          }

          @Override
          public void exit() {}
        };

    setCurrentState(DISABLED);
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
  }

  @Override
  public void outputPeriodic() {

    // Log flywheel speed in RPM
    Logger.recordOutput("Shooter/FlywheelSpeedRPM", getVelocityRPM());

    Logger.recordOutput("Shooter/TargetFeed", currSetpoints.feederVel_rps);
    Logger.recordOutput("Shooter/TargetFly", currSetpoints.flywheel_rps);
    Logger.recordOutput("Shooter/TargetTurr", currSetpoints.turretPos_r);
    Logger.recordOutput("Shooter/TargetPivot", currSetpoints.pivotPos_r);

    io.setFeederVel(currSetpoints.feederVel_rps);
    io.setFlywheelVel(currSetpoints.flywheel_rps);
    io.setTurretPos(currSetpoints.turretPos_r);
    io.setPivotPos(currSetpoints.pivotPos_r);
  }

  /** Stops Everything */
  public void stop() {
    queueSetpoints(new Setpoints(0, 0));
    io.stop();
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

    return 60 * (inputs.flywheelVel_rps);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    io.setFlywheelVolts(volts);
  }

  /**
   * Gives pose for speaker adjusted for alliance 
   * @return
   */
  private static Pose3d getSpeakerPose(){
    return !DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Red ? new Pose3d(0.5,5.5,7,new Rotation3d()) : new Pose3d(0.5,5.5,7,new Rotation3d());
  }

  /**
   * Moves the pose of the speaker slightly based on vector to target
   * @param botpose 3D botpose where height is shooter pivot height
   * @param speakerPose3d 3D pose of speaker where height is bottom of speaker hole, xy is center of speaker
   * @return
   */

   //TODO: do later
  private Pose3d transformSpeakerPoseFromBotpose(Pose2d botpose, Pose3d speakerPose3d){
    return new Pose3d();
  }

  /**
   * The move and shoot code, returns the moved target to aim at
   * @param botpose 3D botpose where height is shooter pivot height
   * @param field_relSpeeds Field relative velocity of bot
   * @param target 3D pose of target to shoot at
   * @return
   */
  private Pose3d transformTargetPoseFromFieldRelativeVelocity(Pose3d botpose, ChassisSpeeds field_relSpeeds, Pose3d target){

    double[] velocityVector = new double[]{field_relSpeeds.vxMetersPerSecond,field_relSpeeds.vyMetersPerSecond};
    Transform3d difference = botpose.minus(target);
    double[] speakerDifferenceVector = new double[]{difference.getX(),difference.getY(),difference.getZ()};

    double[] shoot_vector = new double[]{speakerDifferenceVector[0] - velocityVector[0], speakerDifferenceVector[1] - velocityVector[1]};

    return new Pose3d(shoot_vector[0],shoot_vector[1],target.getZ(),new Rotation3d());
  }

  /**
   * 
   * @param botpose 3D botpose where height is shooter pivot height
   * @param target 3D pose of target to shoot at
   * @return
   */
  private Setpoints calcutateSetpointsForPose(Pose3d botpose, Pose3d target){


    // account for swerve movement
    Pose2d pose = Drive.getInstance().getPose();
    Pose3d transformed_pose = transformTargetPoseFromFieldRelativeVelocity(botpose, Drive.getInstance().getChassisSpeeds(), target);
    return new Setpoints(FLYWHEEL_RAD_PER_SEC, Math.atan2(transformed_pose.getY() - pose.getY(), transformed_pose.getX() - pose.getX()), Math.atan((HEIGHT_DIFFERENCE) / transformed_pose.toPose2d().getTranslation().getDistance(SPEAKER_POSE.toPose2d().getTranslation())));
  }

  /** Returns the required hardware states to be able to shoot in speaker */
  public Setpoints getStateToSpeaker() {

    // check if moving

    Drive drive = Drive.getInstance();

    ChassisSpeeds speeds = drive.getChassisSpeeds();
    Pose2d pose2d = Drive.getInstance().getPose();

    double distanceToSpeaker =
        pose2d.getTranslation().getDistance(SPEAKER_POSE.toPose2d().getTranslation());

    // TODO: MAKE ACTUALLY WORK

    // check if moving
    if (speeds.vyMetersPerSecond > 0
        || speeds.vxMetersPerSecond > 0 && speeds.omegaRadiansPerSecond == 0) {
      double[] velocityVector = new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond};

      Transform3d difference =
          new Pose3d(
                  pose2d.getX() + velocityVector[0] * TIME_TO_SHOOT,
                  pose2d.getY() + velocityVector[1] * TIME_TO_SHOOT,
                  0,
                  new Rotation3d())
              .minus(SPEAKER_POSE);
      double[] speakerDifferenceVector =
          new double[] {difference.getX(), difference.getY(), difference.getZ()};

      double[] shoot_vector =
          new double[] {
            speakerDifferenceVector[0] - velocityVector[0],
            speakerDifferenceVector[1] - velocityVector[1]
          };

      double shoot_heading = Math.atan2(shoot_vector[1], shoot_vector[0]); // might not work
      double shoot_speed = Math.sqrt(Math.pow(shoot_vector[0], 2) + Math.pow(shoot_vector[1], 2));

      double shoot_speed_rad_per_sec = shoot_speed * FLYWHEEL_RADIUS;
      double launch_angle = Math.atan((HEIGHT_DIFFERENCE) / distanceToSpeaker);
      return new Setpoints(shoot_speed_rad_per_sec, shoot_heading, launch_angle);
    }

    // needs to be changed if the angle to hit target at is > 0
    double angle = Math.atan((HEIGHT_DIFFERENCE) / distanceToSpeaker);

    // needs to be rotation from the thing
    double turretPosition =
        Math.atan((SPEAKER_POSE.getY() - pose2d.getY()) / (SPEAKER_POSE.getX() - pose2d.getX()));

    return new Setpoints(FLYWHEEL_RAD_PER_SEC, turretPosition, angle);
  }
}
