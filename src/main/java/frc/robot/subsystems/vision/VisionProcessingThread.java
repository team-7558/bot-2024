package frc.robot.subsystems.vision;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionProcessingThread extends Thread {

  VisionIOPhoton camera;

  private double lastTimestamp = 0;

  public VisionProcessingThread(VisionIOPhoton camera) {
    setDaemon(true);
    setName("testingl");
    this.camera = camera;
  }

  @Override
  public void run() {
    while (true) {
      try {
        if (camera.camera.isConnected()) {
          PhotonPipelineResult latestResult = camera.camera.getLatestResult();
          if (lastTimestamp == latestResult.getTimestampSeconds())
            continue; // dont process the same frame twice

          if (latestResult.hasTargets()) {
            Optional<EstimatedRobotPose> poseOptional = camera.poseEstimator.update(latestResult);
            if (poseOptional.isPresent()) {
              if(latestResult.getBestTarget().getPoseAmbiguity() > 0.2) return; // as per photonvision's reccomendations
              EstimatedRobotPose estimatedPose = poseOptional.get();
              Logger.recordOutput("Vision/3DPose", estimatedPose.estimatedPose);
              camera.poses[camera.poseIndex] = estimatedPose.estimatedPose;
              camera.poseTimestamps[camera.poseIndex] = estimatedPose.timestampSeconds;
              camera.tids[camera.poseIndex] = estimatedPose.targetsUsed.get(0).getFiducialId();
              camera.poseIndex++;
              camera.recentResult = latestResult;
              lastTimestamp = estimatedPose.timestampSeconds;
            }
          }
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }
}
