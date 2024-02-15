package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import frc.robot.subsystems.vision.Vision.TimestampedPose;

import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

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
  private final Transform3d transform;
  private final List<TimestampedPose> poses = new ArrayList<>();
  private final double lastTimestamp = 0;

  public VisionIOPhoton(String camname, Transform3d camToRobot) {
    this.camera = new PhotonCamera(camname);

    // this is a listener for any changes on the photonvision networktable. might need to change this later. this is async 
    // maybe change this to just a while true loop in a thread 
    this.camera.getCameraTable().addListener(EnumSet.of(Kind.kValueAll),new TableEventListener() {

      @Override
      public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        PhotonPipelineResult latestResult = camera.getLatestResult();
        if(lastTimestamp == latestResult.getTimestampSeconds()) return; // dont process the same frame twice
        
        if (latestResult.hasTargets()) {
          Optional<EstimatedRobotPose> poseOptional = poseEstimator.update(latestResult);
          if(poseOptional.isPresent()) {
            EstimatedRobotPose estimatedPose = poseOptional.get();
            poses.add(new TimestampedPose(estimatedPose.estimatedPose.toPose2d(),estimatedPose.timestampSeconds));
          }
        }
      }
      
    });
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
    PhotonPipelineResult latestResult = camera.getLatestResult();
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

  @Override
  public void setPipeline(int pipelineID) {
    camera.setPipelineIndex(pipelineID);
  }

  @Override
  public Transform3d getTransform() {
    return transform;
  }
}
