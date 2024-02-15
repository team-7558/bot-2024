package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Traj {

    private enum TrajType{
        MOVING,
        STAYING
    }

    final TrajType type;

    private PathPlannerTrajectory traj;
    private State initState;

    
    public Traj(String filename, boolean isChoreo, ChassisSpeeds startingSpeeds, Rotation2d startingRotation){
        type = TrajType.MOVING;
        PathPlannerPath pppath = isChoreo ? PathPlannerPath.fromChoreoTrajectory(filename) : PathPlannerPath.fromPathFile(filename);
        this.traj = pppath.getTrajectory(startingSpeeds, startingRotation);
        initState = traj.getInitialState();
    }

    public Traj(double delay_s, State initState){
        type = TrajType.STAYING;
        this.initState = initState;
        this.initState.timeSeconds = delay_s;
        this.initState.velocityMps = 0;
        this.initState.accelerationMpsSq = 0;
        this.initState.headingAngularVelocityRps = 0;
        this.initState.holonomicAngularVelocityRps = Optional.of(0.0);
        this.initState.curvatureRadPerMeter = 0;

    }

    public double endTime(){
        if(type == TrajType.MOVING){
            return traj.getTotalTimeSeconds();
        } else {
            return initState.timeSeconds;
        }
    }

    public State sample(double time){
        if(type == TrajType.MOVING){
            return traj.sample(time);
        } else {
            return initState;
        }
    }

    public State getInitState(){
        return initState;
    }

    public State getEndState(){
        if(type == TrajType.MOVING){
            return traj.getEndState();
        } else {
            return initState;
        }
    }

}
