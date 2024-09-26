package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class Vision {

  private Vision instance;

  private VisionIO3G visionIO;
  private VisionIO3GInputsAutoLogged visionInputs;

  public Pose2d getPose() {
    return visionInputs.pose;
  }

  // TODO: FINISH LITERALLY EVERYTHING LATER!!!!!!!!!!!!!!!!!!!!!!!!!

  public Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
      return instance;
    } else {
      return instance;
    }
  }
}
