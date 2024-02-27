package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.G;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter.Setpoints;
import frc.robot.subsystems.shooter.Shooter.TargetMode;
import org.littletonrobotics.junction.Logger;

public class ShotLogger {

  private static final boolean ENABLED = false;

  public static final void log(Setpoints target, TargetMode targetMode) {
    if (ENABLED) {
      long uuid = Preferences.getLong("shotUUID", -1) + 1;
      String uuidStrT = "SL/" + uuid + "/";

      Logger.recordOutput(uuidStrT + "Flywheel", target.flywheel_rps);
      Logger.recordOutput(uuidStrT + "Feeder", target.feederVel_rps);
      Logger.recordOutput(uuidStrT + "Turret", target.turretPos_r);
      Logger.recordOutput(uuidStrT + "Pivot", target.pivotPos_r);
      Logger.recordOutput(uuidStrT + "TargetMode", targetMode);
      Logger.recordOutput(uuidStrT + "Pose", Drive.getInstance().getPose());
      Logger.recordOutput(uuidStrT + "Speeds", Drive.getInstance().getFieldRelativeSpeeds());
      Logger.recordOutput(uuidStrT + "isRed", G.isRedAlliance());

      Preferences.setLong("shotUUID", uuid);
    }
  }
}
