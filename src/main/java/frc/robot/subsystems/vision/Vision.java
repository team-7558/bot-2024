package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
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

  public boolean shouldUseHighRes(int camera, Pose2d currentPose) {
    boolean canSeeAnyTags = false;
    for (AprilTag tag : Drive.getInstance().getFieldLayout().getTags()) {
      Pose2d tagpose = tag.pose.toPose2d();
      boolean canSee =
          canSeeTarget(currentPose, new Transform2d(0, 0, new Rotation2d()), tagpose, 4.0);
      canSeeAnyTags = canSee;
      System.out.println(canSee);
      if (canSee) return false;
    }
    // cant see any so raise
    return !canSeeAnyTags;
  }

  public static boolean canSeeTarget(
      Pose2d robotPose, Transform2d cameraPose, Pose2d targetPose, double d) {

    Pose2d vo = robotPose.plus(cameraPose);

    Logger.recordOutput("Vision/camerapose", vo);

    double fov = 1.570; // TODO: TUNE rad;

    double theta1 = vo.getRotation().getRadians() - (fov * 0.5);
    double theta2 = vo.getRotation().getRadians() + (fov * 0.5);

    double v1x = d * Math.cos(theta1);
    double v1y = d * Math.sin(theta1);

    double v2x = d * Math.cos(theta2);
    double v2y = d * Math.sin(theta2);

    Translation2d v2 = new Translation2d(v2x, v2y);
    Translation2d v1 = new Translation2d(v1x, v1y);

    Logger.recordOutput("Vision/Vectors/v1", new Pose2d[] {vo, new Pose2d(v1, new Rotation2d())});
    Logger.recordOutput("Vision/Vectors/v2", new Pose2d[] {vo, new Pose2d(v2, new Rotation2d())});

    double a =
        ((determinate(targetPose.getTranslation(), v1) - determinate(vo.getTranslation(), v1))
            / determinate(v2, v1));

    double b =
        -((determinate(targetPose.getTranslation(), v2) - determinate(vo.getTranslation(), v2))
            / determinate(v2, v1));

    return a > 0 && b > 0 && (a + b) < 1;
  }

  private static double determinate(Translation2d i, Translation2d j) {
    return (i.getX() * j.getY()) - (j.getX() * j.getY());
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
        // VisionIOSim camzero =
        //     new VisionIOSim("camera1", new Transform3d(0.3, 0.1, 0.3, new Rotation3d(0, 90,
        // 90)));
        // VisionIOSim camone =
        //     new VisionIOSim("camera1", new Transform3d(-0.3, 0.1, 0.3, new Rotation3d(0, 90,
        // 90)));
        // VisionIOSim camtwo =
        //     new VisionIOSim("camera1", new Transform3d(0.3, 0.1, -0.3, new Rotation3d(0, 90,
        // 0)));
        // VisionIOSim camthree =
        //     new VisionIOSim("camera1", new Transform3d(-0.3, 0.1, -0.3, new Rotation3d(0, 90,
        // 0)));
        return new Vision(new VisionIO() {});

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
