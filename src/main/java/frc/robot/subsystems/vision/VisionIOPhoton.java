package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import java.util.Arrays;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements VisionIO {

  public final PhotonCamera camera;
  public final PhotonPoseEstimator poseEstimator;
  public AprilTagFieldLayout fieldLayout = null;
  public final Transform3d transform;
  public double[] poseTimestamps = new double[100];
  public Pose2d[] poses = new Pose2d[100];
  public PhotonPipelineResult recentResult;
  public int poseIndex = 0;
  public final double lastTimestamp = 0;
  private final VisionProcessingThread thread;

  public VisionIOPhoton(String camname, Transform3d camToRobot) {
    this.camera = new PhotonCamera(camname);
    Arrays.fill(poses, new Pose2d());
    thread = new VisionProcessingThread(this);
    thread.start();

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
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camToRobot);
    this.transform = camToRobot;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.poses = poses;
    inputs.poseTimestamps = poseTimestamps;
    if (recentResult != null) {
      PhotonPipelineResult latestResult = recentResult;
      if (latestResult.hasTargets()) {
        PhotonTrackedTarget target = latestResult.getBestTarget();
        inputs.tagID = target.getFiducialId();
        inputs.xOffset = target.getYaw();
        inputs.yOffset = target.getPitch();
        inputs.pipelineID = camera.getPipelineIndex();
        inputs.timestamp = latestResult.getTimestampSeconds();
        inputs.latency = latestResult.getLatencyMillis();
      }
    }
    poseTimestamps = new double[100];
    poseIndex = 0;
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
