package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface ApriltagIO {

  @AutoLog
  public static class ApriltagIOInputs {
    public int tagID = -1;
    public int pipelineID = 0;
    public Pose3d poses = new Pose3d();
    public double timestamp = 0;
    public double tid = 0;
    public double latency = -1;
  }

  public default void updateInputs(ApriltagIOInputs inputs) {}

  /**
   * Sets the current pipeline based on the ID.
   *
   * @param pipelineID
   */
  public default void setPipeline(int pipelineID) {}

  public default Transform3d getTransform() {
    return new Transform3d();
  }
}
