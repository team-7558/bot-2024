package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private VisionIO cameras[];
  private VisionIOInputsAutoLogged[] visionInputs;

  public Vision(VisionIO... cameras) {
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

  public static Vision getInstance() {
    switch (Constants.currentMode) {
      case REAL:
        VisionIO cam0 =
            new VisionIOPhoton("camera1", new Transform3d()); // TODO: update transform & name
        // VisionIO cam1 =
        //     new VisionIOPhoton("camera2", new Transform3d()); // TODO: update transform & name
        // VisionIO cam2 =
        //     new VisionIOPhoton("camera3", new Transform3d()); // TODO: update transform & name
        // VisionIO cam3 =
        //     new VisionIOPhoton("camera4", new Transform3d()); // TODO: update transform & name
        // VisionIO limelight = new VisionIOLimelight("limelight"); // TODO: update name
        Vision v = new Vision(cam0);
        return v;
      case SIM:
        VisionIOSim camzero =
            new VisionIOSim("camera1", new Transform3d(0.3, 0.1, 0.3, new Rotation3d(0, 90, 90)));
        VisionIOSim camone =
            new VisionIOSim("camera1", new Transform3d(-0.3, 0.1, 0.3, new Rotation3d(0, 90, 90)));
        VisionIOSim camtwo =
            new VisionIOSim("camera1", new Transform3d(0.3, 0.1, -0.3, new Rotation3d(0, 90, 0)));
        VisionIOSim camthree =
            new VisionIOSim("camera1", new Transform3d(-0.3, 0.1, -0.3, new Rotation3d(0, 90, 0)));
        return new Vision(camzero, camone, camtwo, camthree);

      case REPLAY:
        // idk yet

      default:
        // nothing yet
        return new Vision();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(visionInputs[i]);
      Logger.processInputs("Vision/Camera" + i + "/Inputs", visionInputs[i]);
    }
  }
}
