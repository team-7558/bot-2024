package frc.robot.subsystems.vision;

import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionProcessingThread extends Thread {

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
            Drive.getInstance()
                .addToPoseEstimator(
                    estimatedPose.estimatedPose.toPose2d(), latestResult.getTimestampSeconds());
            camera.recentResult = latestResult;
          }
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }
}
