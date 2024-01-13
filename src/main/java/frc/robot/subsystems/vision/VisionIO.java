package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    // TODO: might need more stuff idk
    public double xOffset = 0;
    public double yOffset = 0;
    public int tagID = -1;
    public int pipelineID = 0;
    public Pose2d pose = new Pose2d();
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
}
