package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Trajchain {
    

    private List<Traj> trajs;

    private double chainSize;

    private Pose2d startingPose;

    private List<Double> trajTimes;
    private double totalTime_s;


    private boolean generated = false;

    public Trajchain(){
        trajs = new ArrayList<>();
        trajTimes = new ArrayList<>();
        chainSize = 0;
        totalTime_s = 0;
    }

    public Trajchain append(String filename, boolean isChoreo){
        Traj t = new Traj(filename, isChoreo, new ChassisSpeeds(), new Rotation2d());
        trajs.add(t);
        totalTime_s += t.endTime();
        trajTimes.add(totalTime_s);
        return this;
    }

    public Trajchain append(double delay_s){
        Traj t = new Traj(delay_s, new State());
        trajs.add(t);
        totalTime_s += t.endTime();
        trajTimes.add(totalTime_s);
        chainSize++;
        return this;
    }

    public void generate(){
        int i = 0;
        while(startingPose == null){
            
        }
    }


}
