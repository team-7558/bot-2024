package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO3G {

  @AutoLog
  public static class VisionIO3GInputs {

    public Pose2d pose = new Pose2d();
    public double timestamp = 0;

  }

  public default void setPipeline(double pl) {

  }


  public default void takeSS() {
    // fill later
  }
}
