package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements ApriltagIO {

  public final PhotonCamera camera;
  public final PhotonPoseEstimator poseEstimator;
  public AprilTagFieldLayout fieldLayout = null;
  public final Transform3d transform;
  public PhotonPipelineResult recentResult;
  private final VisionProcessingThread thread;

  public VisionIOPhoton(String camname, Transform3d camToRobot) {
    this.camera = new PhotonCamera(camname);
    thread = new VisionProcessingThread(this);

    // this is a listener for any changes on the photonvision networktable. might need to change
    // this later. this is async
    // maybe change this to just a while true loop in a thread
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      System.out.println("exception");
      e.printStackTrace();
    }
    this.poseEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camToRobot);
    this.transform = camToRobot;
  }

  @Override
  public void updateInputs(ApriltagIOInputs inputs) {

    if (recentResult != null) {
      PhotonPipelineResult latestResult = recentResult;
      if (latestResult.hasTargets()) {
        PhotonTrackedTarget target = latestResult.getBestTarget();
        inputs.tagID = target.getFiducialId();
        inputs.pipelineID = camera.getPipelineIndex();
        inputs.timestamp = latestResult.getTimestampSeconds();
        inputs.latency = latestResult.getLatencyMillis();
      }
    }
  }

  @Override
  public void setPipeline(int pipelineID) {
    camera.setPipelineIndex(pipelineID);
  }

  @Override
  public Transform3d getTransform() {
    return transform;
  }
}
