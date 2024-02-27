package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.subsystems.shooter.Shooter.Setpoints;
import org.littletonrobotics.junction.Logger;

public class ShotLogger {

  public static final void log(Setpoints target, Pose2d pose) {
    long uuid = Preferences.getLong("shotUUID", -1) + 1;
    String uuidStrT = "SL/" + uuid + "/";

    Logger.recordOutput(uuidStrT + "Flywheel", target.flywheel_rps);
    Logger.recordOutput(uuidStrT + "Feeder", target.feederVel_rps);
    Logger.recordOutput(uuidStrT + "Turret", target.turretPos_r);
    Logger.recordOutput(uuidStrT + "Pivot", target.pivotPos_r);
    Logger.recordOutput(uuidStrT + "Pose", pose);

    Preferences.setLong("shotUUID", uuid);
  }
}
