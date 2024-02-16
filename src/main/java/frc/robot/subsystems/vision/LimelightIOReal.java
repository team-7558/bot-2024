package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.LimelightIO.LimelightIOInputs;

public class LimelightIOReal implements LimelightIO {

  NetworkTable limelight;

  public LimelightIOReal(String limelightName) {
    limelight = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    inputs.tx = limelight.getEntry("tx").getDouble(0);
    inputs.ty = limelight.getEntry("ty").getDouble(0);
    inputs.pl = (int) limelight.getEntry("pl").getDouble(-1);
    inputs.tl = limelight.getEntry("tl").getDouble(-1);
  }

  @Override
  public void setPipeline(double pipelineID) {
    limelight.getEntry("pl").setDouble(pipelineID);
  }
}
