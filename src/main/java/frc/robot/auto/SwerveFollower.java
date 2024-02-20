package frc.robot.auto;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class SwerveFollower {

    private final PPHolonomicDriveController con;

    private final IFollowable toFollow;

    private final Timer t;

    public SwerveFollower(IFollowable toFollow){
        this.toFollow = toFollow;

        con = new PPHolonomicDriveController(Drive.HPFG.translationConstants, Drive.HPFG.rotationConstants, 
                Constants.globalDelta_sec, Drive.MAX_LINEAR_SPEED, Drive.DRIVE_BASE_RADIUS);

        
        t = new Timer();
    }

    public void start(){

    }


}
