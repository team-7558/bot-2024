
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private VisionIO cameras[] = new VisionIO[4];
  private PoseEs

  public Vision() {
    cameras[0] = new VisionIOPhoton("example", new Transform3d()); // TODO: fill transform3d when testing
  }

  @Override
  public void periodic() {
  }

  public Pose2d getPose() {
    return 
  }
  
  public static Vision getInstance() {
    return new Vision();
  }
}
