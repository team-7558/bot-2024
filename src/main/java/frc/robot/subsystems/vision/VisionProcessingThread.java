package frc.robot.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionProcessingThread extends Thread {

  public static final int MAX_FPS = 60;
  public static final long SLEEP_TIME = 1000 / 60;

  VisionIOPhoton camera;

  public VisionProcessingThread(VisionIOPhoton camera) {
    this.camera = camera;
    setDaemon(true);
    start();
  }

  @Override
  public void run() {
    while (true) {

      try {
        if (camera.camera.isConnected()) {
          PhotonPipelineResult latestResult = camera.camera.getLatestResult();
          Optional<EstimatedRobotPose> poseOptional = camera.poseEstimator.update(latestResult);
          if (poseOptional.isPresent()) {
            EstimatedRobotPose estimatedPose = poseOptional.get();

            camera.timestamps.offer(latestResult.getTimestampSeconds());

            camera.poses.offer(estimatedPose.estimatedPose);

            Integer[] tids = new Integer[camera.camera.getLatestResult().getTargets().size()];

            int i = 0;
            for (PhotonTrackedTarget target : camera.camera.getLatestResult().getTargets()) {
              tids[i] = target.getFiducialId();
              i++;
            }

            camera.tids.offer(tids);

            camera.ambiguity.offer(latestResult.getBestTarget().getPoseAmbiguity());
          }
        }
        // Thread.sleep(SLEEP_TIME);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }
}
