package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  // default pipeline, tracking apriltags at high FPS and low res.
  public static final int QUICK_PIPELINE_ID = 1;

  // secondary pipeline, tracking apriltags at high res and low FPS.
  public static final int CLEAR_PIPELINE_ID = 0;

  // Angle of camera horizontal FOV
  public static final int CAM_FOV_DEG = 120;

  // Distance for quick
  public static final double QUICK_DISTANCE_M = 4.0;

  // Distance for clear
  public static final double CLEAR_DISTANCE_M = QUICK_DISTANCE_M + 0.1;

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

      VisionIO cam0, cam1, cam2, cam3;

      switch (Constants.currentMode) {
        case REAL:
          cam0 = new VisionIOPhoton("camera0", new Transform3d()); // TODO: update transform & name
          // VisionIO cam1 =
          //     new VisionIOPhoton("camera2", new Transform3d()); // TODO: update transform & name
          // VisionIO cam2 =
          //     new VisionIOPhoton("camera3", new Transform3d()); // TODO: update transform & name
          // VisionIO cam3 =
          //     new VisionIOPhoton("camera4", new Transform3d()); // TODO: update transform & name
          // VisionIO limelight = new VisionIOLimelight("limelight"); // TODO: update name
          instance = new Vision(cam0);
          break;
        case SIM:
          cam0 = new VisionIOSim("camera0", new Transform3d());
          // VisionIOSim camone =
          //     new VisionIOSim("camera1", new Transform3d(-0.3, 0.1, 0.3, new Rotation3d(0, 90,
          // 90)));
          // VisionIOSim camtwo =
          //     new VisionIOSim("camera1", new Transform3d(0.3, 0.1, -0.3, new Rotation3d(0, 90,
          // 0)));
          // VisionIOSim camthree =
          //     new VisionIOSim("camera1", new Transform3d(-0.3, 0.1, -0.3, new Rotation3d(0, 90,
          // 0)));
          instance = new Vision(cam0);
          break;
        case REPLAY:
          // idk yet

        default:
          // nothing yet
          instance = new Vision(new VisionIO() {});
      }
    }

    return instance;
  }

  private VisionIO cameras[];
  private VisionIOInputsAutoLogged[] visionInputs;

  private Vision(VisionIO... cameras) {
    this.cameras = cameras;
    this.visionInputs =
        new VisionIOInputsAutoLogged[] {
          new VisionIOInputsAutoLogged(),
          new VisionIOInputsAutoLogged(),
          new VisionIOInputsAutoLogged(),
          new VisionIOInputsAutoLogged()
        };
  }

  public int getCameras() {
    return cameras.length;
  }

  public Pose2d[] getPoses() {
    Pose2d[] poses = new Pose2d[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      poses[i] = visionInputs[i].pose;
    }
    return poses;
  }

  public void setPipeline(int camera, int pipeline) {
    cameras[camera].setPipeline(pipeline);
  }

  public int getPipeline(int camera) {
    return visionInputs[camera].pipelineID;
  }

  public double getXOffset(int camera) {
    return visionInputs[camera].xOffset;
  }

  public double getYOffset(int camera) {
    return visionInputs[camera].yOffset;
  }

  public int getTagID(int camera) {
    return visionInputs[camera].tagID;
  }

  public Pose2d getPose(int camera) {
    return visionInputs[camera].pose;
  }

  public double getTimestamp(int camera) {
    return visionInputs[camera].timestamp;
  }

  public double getLatency(int camera) {
    return visionInputs[camera].latency;
  }

  public boolean hasTagInView() {
    for (int i = 0; i < visionInputs.length; i++) {
      VisionIOInputsAutoLogged input = visionInputs[i];
      if (input.tagID != 1) {
        return true;
      }
    }
    return false;
  }

  public void managePipelines(int camID, Pose2d botpose) {
    VisionIO cam = cameras[camID];
    boolean shouldSwitchToQuick = false;
    boolean shouldSwitchToClear = false;

    Transform3d camTransform3d = cam.getTransform();
    Transform2d camInBotSpace =
        new Transform2d(
            camTransform3d.getTranslation().toTranslation2d(),
            camTransform3d.getRotation().toRotation2d());

    Pose2d v0 = botpose.plus(camInBotSpace);

    Logger.recordOutput("Vision/campose", v0);

    double fov = Units.degreesToRadians(CAM_FOV_DEG); // TODO: TUNE rad;

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

    double detv0v1 = Util.det(v0.getTranslation(), v1);
    double detv0v2 = Util.det(v0.getTranslation(), v2);
    double detv1v2_inverted = 1.0 / Util.det(v1, v2);

    for (AprilTag tag : AT_MAP.getTags()) {
      Pose2d tagpose = tag.pose.toPose2d();
      double detvv1 = Util.det(tagpose.getTranslation(), v1);
      double detvv2 = Util.det(tagpose.getTranslation(), v2);

      double deltaX = tagpose.getX() - v0.getX();
      double deltaY = tagpose.getY() - v0.getY();

      double deltaXsquared = (deltaX) * (deltaX);
      double deltaYsquared = (deltaY) * (deltaY);

      if ((deltaXsquared + deltaYsquared) <= (QUICK_DISTANCE_M * QUICK_DISTANCE_M)) {
        double atan2 = Math.atan2(deltaY, deltaX);
        if (atan2 <= theta2 && atan2 >= theta1) {
          shouldSwitchToQuick = true;
          break;
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
      // cam.setPipeline(CLEAR_PIPELINE_ID);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(visionInputs[i]);
      Logger.processInputs("Vision/Camera" + i + "/Inputs", visionInputs[i]);

      managePipelines(i, Drive.getInstance().getPose());
    }
  }
}
