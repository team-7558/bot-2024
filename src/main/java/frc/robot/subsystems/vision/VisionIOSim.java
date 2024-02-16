package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSim implements VisionIO {
  private final PhotonCameraSim camera;
  private final PhotonPoseEstimator poseEstimator;
  private final Transform3d camToRobot;
  private AprilTagFieldLayout fieldLayout = null;
  private final List<VisionTargetSim> apriltags;

  public VisionIOSim(String camname, Transform3d camToRobot) {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    var cam = new PhotonCamera(camname);
    var prop = new SimCameraProperties();
    prop.setFPS(60);
    prop.setAvgLatencyMs(40);
    prop.setExposureTimeMs(10);
    prop.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    prop.setCalibError(0.2, 0.05);

    this.camera = new PhotonCameraSim(cam, prop);
    this.camera.prop.setAvgLatencyMs(40);
    this.camera.prop.setFPS(60);
    this.camera.setMaxSightRange(4);

    ArrayList<VisionTargetSim> targets = new ArrayList<>();
    for (AprilTag tag : fieldLayout.getTags()) {
      targets.add(new VisionTargetSim(tag.pose, TargetModel.kAprilTag36h11));
    }
    this.apriltags = targets;

    this.camToRobot = camToRobot;

    this.poseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, camToRobot);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Pose2d odometryPose = Drive.getInstance().getPose();
    Pose3d cameraPose =
        new Pose3d(
            odometryPose.getX() + camToRobot.getX(),
            0 + camToRobot.getY(),
            odometryPose.getY() + camToRobot.getZ(),
            new Rotation3d(
                camToRobot.getRotation().getX(),
                odometryPose.getRotation().getTan() + camToRobot.getRotation().getY(),
                odometryPose.getRotation().getSin() + camToRobot.getRotation().getZ()));
    PhotonPipelineResult latestResult =
        camera.process(camera.prop.getAvgLatencyMs(), cameraPose, this.apriltags);
    if (latestResult.hasTargets()) {
      PhotonTrackedTarget target = latestResult.getBestTarget();
      inputs.tagID = target.getFiducialId();
      inputs.xOffset = target.getYaw();
      inputs.yOffset = target.getPitch();
      // poseEstimator
      //     .update(latestResult)
      //     .ifPresent(
      //         (pose) -> {
      //           inputs.poses.add(
      //               new TimestampedPose(pose.estimatedPose.toPose2d(),
      // Timer.getFPGATimestamp()));
      //         });

      // sim doenst work anyways idc about this
      inputs.timestamp = latestResult.getTimestampSeconds();
      inputs.latency = latestResult.getLatencyMillis();
    }
  }

  @Override
  public void setPipeline(int pipelineID) {
    if (pipelineID == 0) {
      this.camera.prop.setAvgLatencyMs(40);
      this.camera.prop.setFPS(60);
      this.camera.setMaxSightRange(4);
    } else if (pipelineID == 1) {
      this.camera.prop.setAvgLatencyMs(60);
      this.camera.prop.setFPS(20);
      this.camera.setMaxSightRange(5.5);
      this.camera.prop.setCalibration(1920, 480, Rotation2d.fromDegrees(100));
    }
  }

  @Override
  public Transform3d getTransform() {
    return camToRobot;
  }
}
