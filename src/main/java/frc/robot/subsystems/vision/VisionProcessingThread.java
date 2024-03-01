package frc.robot.subsystems.vision;

import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionProcessingThread extends Thread {


  public static final int MAX_FPS = 60;
  public static final long SLEEP_TIME = 1000 / 60;

  VisionIOPhoton camera;

  public VisionProcessingThread(VisionIOPhoton camera) {
    this.camera = camera;
  }

  @Override
  public void run() {
    while (true) {
      try {
        if (camera.camera.isConnected()) {
          PhotonPipelineResult latestResult = camera.camera.getLatestResult();
          camera.poseEstimator.setReferencePose(Drive.getInstance().getPoseEstimatorPose());
          Optional<EstimatedRobotPose> poseOptional = camera.poseEstimator.update(latestResult);
          if (poseOptional.isPresent()) {

            EstimatedRobotPose estimatedPose = poseOptional.get();

            double distanceSums = 0;
            for(PhotonTrackedTarget tag : latestResult.targets) {
              distanceSums += Vision.AT_MAP.getTagPose(tag.getFiducialId()).get().toPose2d().getTranslation().getDistance(Drive.getInstance().getPoseEstimatorPose().getTranslation());
            }
            double distance = distanceSums / latestResult.targets.size();

            Drive.getInstance()
                .addToPoseEstimator(
                    estimatedPose.estimatedPose.toPose2d(), latestResult.getTimestampSeconds(),distance);
            camera.recentResult = latestResult;
          }
        }
        Thread.sleep(SLEEP_TIME);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }
}
