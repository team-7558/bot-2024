package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.io.IOException;
import java.util.Arrays;
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
  public double[] poseTimestamps =
      new double[100]; // TODO: figure out the max array size that we actually need
  public Pose3d[] poses = new Pose3d[100];
  public double[] tids = new double[100];
  public PhotonPipelineResult recentResult;
  public int poseIndex = 0;
  public final double lastTimestamp = 0;
  private final VisionProcessingThread thread;

  public VisionIOPhoton(String camname, Transform3d camToRobot) {
    this.camera = new PhotonCamera(camname);
    Arrays.fill(poses, new Pose3d());
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
  public void updateInputs(ApriltagIOInputs inputs) {

    poseTimestamps = new double[100];
    tids = new double[100];
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

    poseIndex = 0;
    inputs.poses = poses;
    inputs.poseTimestamps = poseTimestamps;
    inputs.tids = tids;
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
