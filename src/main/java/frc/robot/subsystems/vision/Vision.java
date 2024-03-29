package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.PerfTracker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.ApriltagIO.ApriltagIOInputs;
import frc.robot.util.Util;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision {

  // default pipeline, tracking apriltags at high FPS and low res.
  public static final int QUICK_PIPELINE_ID = 1;

  // secondary pipeline, tracking apriltags at high res and low FPS.
  public static final int CLEAR_PIPELINE_ID = 0;

  // Angle of camera horizontal FOV
  public static final double CAM_FOV_RAD = Units.degreesToRadians(120);

  // Angle of apriltag view horizontal FOV
  public static final double AT_FOV_RAD = Units.degreesToRadians(140);

  // Distance for quick
  public static final double QUICK_DISTANCE_M = 1.0;

  // Distance for clear
  public static final double CLEAR_DISTANCE_M = QUICK_DISTANCE_M + 0.1;

  public static final int SPEAKER_LEFT_BLUE = 8, SPEAKER_RIGHT_BLUE = 7;
  public static final int SPEAKER_LEFT_RED = 4, SPEAKER_RIGHT_RED = 3;
  public static final int STAGE_BLUE_1 = 16, STAGE_BLUE_2 = 15, STAGE_BLUE_3 = 14;
  public static final int STAGE_RED_1 = 11, STAGE_RED_2 = 13, STAGE_RED_3 = 12;
  public static final int AMP_RED = 5, AMP_BLUE = 6;
  public static final int SOURCE_RED_1 = 9, SOURCE_RED_2 = 10;
  public static final int SOURCE_BLUE_1 = 1, SOURCE_BLUE_2 = 2;

  public static final double MAX_HEIGHT = 1.5; // m
  public static final double MAX_AMBIGUITY = 0.2; // .2 reccomended by photon

  public static final AprilTagFieldLayout AT_MAP;

  static {
    AprilTagFieldLayout temp;
    try {
      temp = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      temp = null;
      e.printStackTrace();
    }
    AT_MAP = temp;
  }

  private static Vision instance;

  public static Vision getInstance() {
    if (instance == null) {

      VisionIOPhoton cam0, cam1, cam2, cam3;

      switch (Constants.currentMode) {
        case REAL:
          cam0 =
              new VisionIOPhoton(
                  "FR",
                  new Transform3d(
                      -0.1905, // -0.1905
                      0.33, // 0.33
                      -0.508, // -0.508
                      new Rotation3d(
                          Units.degreesToRadians(160), // 45
                          Units.degreesToRadians(-30), // -30
                          Units.degreesToRadians(310)))); // -90
          // cam1 =
          //     new VisionIOPhoton(
          //         "BL",
          //         new Transform3d(
          //             0.1905,
          //             -0.33,

          //             0.508,
          //             new Rotation3d(
          //                 Units.degreesToRadians(45),
          //                 Units.degreesToRadians(30),
          //                 Units.degreesToRadians(0)))); // TODO: needs to be found
          instance = new Vision(cam0);
          break;
        case SIM:
          cam0 = new VisionIOPhoton("camera0", new Transform3d(0, 0, 0, new Rotation3d()));
          instance = new Vision(cam0);
          // no sim
          break;
        case REPLAY:
          // idk yet

        default:
          // nothing yet
          //
      }
    }

    return instance;
  }

  private List<Pose2d> posesToLog;

  private VisionIOPhoton cameras[];
  private ApriltagIOInputs[] visionInputs;

  private Vision(VisionIOPhoton... cameras) {
    this.cameras = cameras;
    this.visionInputs =
        new ApriltagIOInputs[] {new ApriltagIOInputs()
          // new ApriltagIOInputs(),
          // new ApriltagIOInputs()
        };
    posesToLog = new ArrayList<>();
  }

  public boolean hasTagInView() {
    for (int i = 0; i < visionInputs.length; i++) {
      ApriltagIOInputs input = visionInputs[i];
      if (input.tids.length > 0) {
        return true;
      }
    }
    return false;
  }

  public void managePipelines(int camID, Pose2d botpose) {
    ApriltagIO cam = cameras[camID];
    boolean shouldSwitchToQuick = false;
    boolean shouldSwitchToClear = false;

    Transform3d camTransform3d = cam.getTransform();
    Transform2d camInBotSpace =
        new Transform2d(
            camTransform3d.getTranslation().toTranslation2d(),
            camTransform3d.getRotation().toRotation2d());

    Pose2d v0 = botpose.plus(camInBotSpace);

    Logger.recordOutput("Vision/campose", v0);

    double fov = CAM_FOV_RAD;

    double theta1 = v0.getRotation().getRadians() - (fov * 0.5);
    double theta2 = v0.getRotation().getRadians() + (fov * 0.5);

    double v1x = QUICK_DISTANCE_M * Math.cos(theta1);
    double v1y = QUICK_DISTANCE_M * Math.sin(theta1);

    double v2x = QUICK_DISTANCE_M * Math.cos(theta2);
    double v2y = QUICK_DISTANCE_M * Math.sin(theta2);

    Translation2d v2 = new Translation2d(v2x, v2y);
    Translation2d v1 = new Translation2d(v1x, v1y);

    Logger.recordOutput(
        "Vision/Vectors/v1",
        new Pose2d(v1.plus(v0.getTranslation()), Rotation2d.fromRadians(theta1)));
    Logger.recordOutput(
        "Vision/Vectors/v2",
        new Pose2d(v2.plus(v0.getTranslation()), Rotation2d.fromRadians(theta2)));

    for (AprilTag tag : AT_MAP.getTags()) {
      Pose2d tagpose = tag.pose.toPose2d();

      double deltaX = tagpose.getX() - v0.getX();
      double deltaY = tagpose.getY() - v0.getY();

      double vTheta = Math.atan2(deltaY, deltaX);
      double tagTheta = tagpose.getRotation().getRadians() + Math.PI;

      if (Util.isWithinDistanceInclusive(deltaX, deltaY, QUICK_DISTANCE_M)) {
        if (Util.isWithinAngleInclusive(v0.getRotation().getRadians(), vTheta, CAM_FOV_RAD)) {
          if (Util.isWithinAngleInclusive(tagTheta, vTheta, AT_FOV_RAD)) {
            posesToLog.add(tagpose);
            shouldSwitchToQuick = true;
            break; // TODO: reintroduce break for efficiency
          }
        }
      }
    }

    if (shouldSwitchToQuick) {
      Logger.recordOutput("Vision/Heatmap", v0);
      cam.setPipeline(QUICK_PIPELINE_ID);
      Logger.recordOutput("Vision/Camera" + camID + "/Quick?", true);
    } else {
      cam.setPipeline(CLEAR_PIPELINE_ID);
      Logger.recordOutput("Vision/Camera" + camID + "/Quick?", false);
    }

    if (shouldSwitchToClear) {
      cam.setPipeline(CLEAR_PIPELINE_ID);
    }
  }

  public void handleFrameData() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].visionLock.lock();
      cameras[i].updateInputs(visionInputs[i]);
      Logger.processInputs("Vision/Camera" + i + "/Inputs", visionInputs[i]);

      for (int j = 0; j < visionInputs[i].poses.length; j++) {
        try {
          double ambiguity = visionInputs[i].ambiguity[j];
          double timestamp = visionInputs[i].timestamps[j];
          Pose3d pose = visionInputs[i].poses[j];
          int[] tids = visionInputs[i].tids[j];
          if (tids == null) continue;

          // weight bad tags
          double blacklistCoeff = 1;
          for (int tid : tids) {
            if (tid == SPEAKER_LEFT_BLUE
                || tid == SPEAKER_RIGHT_BLUE
                || tid == SPEAKER_LEFT_RED
                || tid == SPEAKER_RIGHT_RED) { // not flat sometimes
              blacklistCoeff += 3;
            } else if (tid == STAGE_BLUE_1
                || tid == STAGE_BLUE_2
                || tid == STAGE_BLUE_3
                || tid == STAGE_RED_1
                || tid == STAGE_RED_2
                || tid == STAGE_RED_3) { // stage sometimes tilts
              blacklistCoeff += 0.5;
            }
          }

          // noise reduction checks

          if (ambiguity > MAX_AMBIGUITY) continue;

          if (pose.getZ() > MAX_HEIGHT)
            continue; // reject impossible pose off the ground (more than hanging)

          if (pose.getX() > AT_MAP.getFieldLength()
              || pose.getX() > AT_MAP.getFieldWidth()
              || pose.getX() < 0
              || pose.getY() < 0) continue; // if outside dont add to pose estimator

          Drive.getInstance()
              .addToPoseEstimator(pose.toPose2d(), timestamp, ambiguity, blacklistCoeff, tids);
        } catch (Exception e) {
        }
      }

      managePipelines(i, Drive.getInstance().getPose());
      cameras[i].timestamps.clear();
      cameras[i].poses.clear();
      cameras[i].tids.clear();
      cameras[i].visionLock.unlock();
    }
  }

  public void periodic() {
    int id = PerfTracker.start("Vision");
    handleFrameData();
    Logger.recordOutput("Vision/TagSet", posesToLog.toArray(new Pose2d[0]));
    PerfTracker.end(id);
  }

  public static class TimestampedPose {

    public Pose2d pose;
    public double timestamp;

    public TimestampedPose(Pose2d pose, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
    }
  }
}
