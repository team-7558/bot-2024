package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {

  NetworkTable limelight;

  public VisionIOLimelight(String limelightName) {
    limelight = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.xOffset = limelight.getEntry("tx").getDouble(0);
    inputs.yOffset = limelight.getEntry("ty").getDouble(0);
    inputs.pipelineID = (int) limelight.getEntry("pl").getDouble(-1);
    inputs.tagID = (int) limelight.getEntry("tid").getDouble(-1);

    double[] botpose_double = limelight.getEntry("botpose").getDoubleArray(new double[7]);
    inputs.timestamp = System.currentTimeMillis() - botpose_double[6];
    // no pose estimation from limelight maybe?? add back later if needed
  }

  @Override
  public void setPipeline(int pipelineID) {
    limelight.getEntry("pl").setDouble(pipelineID);
  }
}
