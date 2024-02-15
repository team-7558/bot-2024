package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision.TimestampedPose;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    // TODO: might need more stuff idk
    public double xOffset = 0;
    public double yOffset = 0;
    public int tagID = -1;
    public int pipelineID = 0;
    public List<TimestampedPose> poses = new ArrayList<>();
    public double timestamp = -1;
    public double latency = -1;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Sets the current pipeline based on the ID.
   *
   * @param pipelineID
   */
  public default void setPipeline(int pipelineID) {}

  public default Transform3d getTransform() {
    return null;
  }
}
