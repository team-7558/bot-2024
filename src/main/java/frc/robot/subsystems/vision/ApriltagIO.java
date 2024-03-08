package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ApriltagIO {

  // DONT ADD THE AUTOLOG HERE
  public static class ApriltagIOInputs implements LoggableInputs {
    public double pipelineID = 0;
    public Pose3d[] poses = new Pose3d[1];
    public double[] timestamps = new double[1];
    public int[][] tids = new int[1][];
    public double[] ambiguity = new double[1];

    @Override
    public void toLog(LogTable table) {
      table.put("PipelineID", pipelineID);
      table.put("Poses", poses);
      table.put("Timestamps", timestamps);

      for (int i = 0; i < tids.length; i++) {
        int[] tid_array = tids[i];
        table.put("Tids" + i, tid_array);
      }
      table.put("Ambiguity", ambiguity);
    }

    @Override
    public void fromLog(LogTable table) {
      pipelineID = table.get("PipelineID", pipelineID);
      poses = table.get("Poses", poses);
      timestamps = table.get("Timestamps", timestamps);
      for (int i = 0; i < tids.length; i++) {
        tids[i] = table.get("Tids" + i, new int[] {});
      }
      ambiguity = table.get("Ambiguity", ambiguity);
    }
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
