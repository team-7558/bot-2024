package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements VisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private AprilTagFieldLayout fieldLayout = null;

  public VisionIOPhoton(String camname, Transform3d robotToCam) {
    this.camera = new PhotonCamera(camname);
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("exception");
      e.printStackTrace();
    }
    this.poseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult latestResult = camera.getLatestResult();
    if (latestResult.hasTargets()) {
      PhotonTrackedTarget target = latestResult.getBestTarget();
      inputs.tagID = target.getFiducialId();
      inputs.xOffset = target.getYaw();
      inputs.yOffset = target.getPitch();
      Optional<EstimatedRobotPose> poseOptional = poseEstimator.update(latestResult);
      if (!poseOptional.isEmpty()) {
        System.out.println("its not null");
        inputs.pose = poseOptional.get().estimatedPose.toPose2d();
      }
      inputs.pipelineID = camera.getPipelineIndex();
      inputs.timestamp = latestResult.getTimestampSeconds();
      inputs.latency = latestResult.getLatencyMillis();
    }
  }

  @Override
  public void setPipeline(int pipelineID) {
    camera.setPipelineIndex(pipelineID);
  }
}
