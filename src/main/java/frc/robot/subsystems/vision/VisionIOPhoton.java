package frc.robot.subsystems.vision;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionIOPhoton implements VisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private AprilTagFieldLayout fieldLayout = null;
  

  public VisionIOPhoton(String camname,Transform3d robotToCam) {
    this.camera = new PhotonCamera(camname);
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    this.poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult latestResult = camera.getLatestResult();
    if(latestResult.hasTargets()) {
      PhotonTrackedTarget target = latestResult.getBestTarget();
      inputs.tagID = target.getFiducialId();
      inputs.xOffset = target.getYaw();
      inputs.yOffset = target.getPitch();
    }
  }

  @Override
  public void setPipeline(double pipelineID) {}
}
