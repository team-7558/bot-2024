package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    // TODO: might need more stuff idk
    public double xOffset;
    public double yOffset;
    public int tagID;
    public double pipelineID;
    public Pose2d pose;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Sets the current pipeline based on the ID.
   *
   * @param pipelineID
   */
  public default void setPipeline(int pipelineID) {}
}
