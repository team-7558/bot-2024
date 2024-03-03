package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class ShotLogger {
    private static final boolean ENABLED = false;

    public static final void log(){
        if(ENABLED){
            long uuid = Preferences.getLong("shotUUID", -1) + 1;

            String logPrefix = "SL/" + uuid;

            Setpoints s = Shooter.getInstance().getMeasuredSetpoints();

            Logger.recordOutput(logPrefix+"/FlyVel", s.flywheel_rps);
            Logger.recordOutput(logPrefix+"/FeedVel", s.feederVel_rps);
            Logger.recordOutput(logPrefix+"/TurretPos", s.turretPos_r);
            Logger.recordOutput(logPrefix+"/PivotPos", s.pivotPos_r);
            Logger.recordOutput(logPrefix+"/Pose", Drive.getInstance().getPose());
            Logger.recordOutput(logPrefix+"/Speeds", Drive.getInstance().getFieldRelativeSpeeds());
            Logger.recordOutput(logPrefix+"/TargetMode", Shooter.getInstance().getTargetMode());

            Preferences.setLong("shotUUID", uuid);
        }
    }
}
