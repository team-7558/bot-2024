package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.PerfTracker;
import frc.robot.subsystems.drive.Drive;
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
  public static final double AT_FOV_RAD = Units.degreesToRadians(140); // TODO: TUNE

  // Distance for quick
  public static final double QUICK_DISTANCE_M = 3.0;

  // Distance for clear
  public static final double CLEAR_DISTANCE_M = QUICK_DISTANCE_M + 0.1;

  public static final AprilTagFieldLayout AT_MAP;

  public static record VisionUpdate(Pose2d pose, double timestamp, double distance) {}

  public static List<VisionUpdate> visionUpdates = new ArrayList<>();

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

      switch (Constants.currentMode) { // TODO: SET BACK TO NORMAL
        case REAL:
          cam0 =
              new VisionIOPhoton(
                  "BL",
                  new Transform3d(
                      0.65,
                      -0.28,
                      -0.3,
                      new Rotation3d(
                          Units.degreesToRadians(-10),
                          Units.degreesToRadians(-10),
                          Units.degreesToRadians(130)))); // TODO: update transform & name later
          cam1 =
              new VisionIOPhoton(
                  "BR",
                  new Transform3d(
                      0.65,
                      0.28,
                      -0.3,
                      new Rotation3d(
                          Units.degreesToRadians(10),
                          Units.degreesToRadians(-10),
                          Units.degreesToRadians(-130))));
          // VisionIO cam1 =7
          //     new VisionIOPhoton("camera2", new Transform3d()); // TODO: update transform & name
          // VisionIO cam2 =
          //     new VisionIOPhoton("camera3", new Transform3d()); // TODO: update transform & name
          // VisionIO cam3 =
          //     new VisionIOPhoton("camera4", new Transform3d()); // TODO: update transform & name
          // LimelightIO limelight = new LimelightIOReal("limelight");
          instance = new Vision(new LimelightIO() {}, cam1);
          break;
        case SIM:
          cam0 =
              new VisionIOPhoton(
                  "camera0",
                  new Transform3d(
                      0, 0, 0, new Rotation3d())); // TODO: update transform & name later
          instance = new Vision(new LimelightIO() {}, cam0);
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

  private LimelightIO limelight;
  public LimelightIOInputsAutoLogged limelightInputs;

  private VisionIOPhoton cameras[];
  private ApriltagIOInputsAutoLogged[] visionInputs;

  private Vision(LimelightIO limelight, VisionIOPhoton... cameras) {
    this.cameras = cameras;
    this.visionInputs =
        new ApriltagIOInputsAutoLogged[] {
          new ApriltagIOInputsAutoLogged(),
          new ApriltagIOInputsAutoLogged(),
          new ApriltagIOInputsAutoLogged(),
          new ApriltagIOInputsAutoLogged()
        };
    this.limelight = limelight;
    this.limelightInputs = new LimelightIOInputsAutoLogged();
    posesToLog = new ArrayList<>();
  }

  public int getCameras() {
    return cameras.length;
  }

  public void setPipeline(int camera, int pipeline) {
    cameras[camera].setPipeline(pipeline);
  }

  public int getPipeline(int camera) {
    return visionInputs[camera].pipelineID;
  }

  public int getTagID(int camera) {
    return visionInputs[camera].tagID;
  }

  public double getTimestamp(int camera) {
    return visionInputs[camera].timestamp;
  }

  public double getLatency(int camera) {
    return visionInputs[camera].latency;
  }

  public boolean hasTagInView() {
    for (int i = 0; i < visionInputs.length; i++) {
      ApriltagIOInputsAutoLogged input = visionInputs[i];
      if (input.tagID != 1) {
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

    double fov = CAM_FOV_RAD; // TODO: TUNE rad;

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
            // break; //TODO: reintroduce break for efficiency
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

    if (shouldSwitchToClear) { // TODO: implement for noise reduction
      cam.setPipeline(CLEAR_PIPELINE_ID);
    }
  }

  public void handleFrameData() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(visionInputs[i]);
      // Logger.processInputs("Vision/Camera" + i + "/Inputs", visionInputs[i]);
      managePipelines(i, Drive.getInstance().getPose());
    }
  }

  public void periodic() {
    // Logger.processInputs("Vision/Limelight", limelightInputs);
    handleFrameData();
    Logger.recordOutput("Vision/TagSet", posesToLog.toArray(new Pose2d[0]));
    posesToLog.clear();
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
