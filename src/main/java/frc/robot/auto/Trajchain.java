package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;

public class Trajchain implements IFollowable{
    

    private List<Traj> trajs;

    private int chainSize, firstPath;

    private Pose2d startingPose;

    private List<Double> startTimes;
    private double totalTime_s;


    private boolean generated = false;

    public Trajchain(){
        trajs = new ArrayList<>();
        startTimes = new ArrayList<>();
        chainSize = 0;
        firstPath = -1;
        totalTime_s = 0;
    }

    public Trajchain append(String filename, boolean isChoreo){
        Traj t = new Traj(filename, isChoreo);
        trajs.add(t);
        chainSize++;
        return this;
    }

    public Trajchain append(double delay_s){
        Traj t = new Traj(delay_s);
        trajs.add(t);
        if(firstPath == -1) firstPath = chainSize;
        chainSize++;
        return this;
    }

    @Override
    public void generate(){
        if(firstPath == -1){
            System.err.println("Path of just waits");
        } else {
            trajs.get(firstPath).generate();
            State initState = trajs.get(firstPath).getInitState();
            startingPose = new Pose2d(initState.positionMeters, initState.targetHolonomicRotation);
            startTimes.add(0.0);
            totalTime_s = 0;
            for(int i = firstPath+1; i < chainSize; i++){
                trajs.get(i).generate(trajs.get(i-1).getEndState());
            }
            for(int i = 0; i < firstPath; i++){
                trajs.get(i).generate(initState);
            }
            for(int i = 1; i < chainSize; i++){
                totalTime_s += trajs.get(i-1).endTime();
                startTimes.add(totalTime_s);
            }

            generated = true;
        }
    }

    @Override
    public boolean isGenerated(){
        return generated;
    }
    
    @Override
    public State sample(double time_s){

        int i = 0;
        while(time_s < startTimes.get(i)) i++;

        return trajs.get(i).sample(time_s - startTimes.get(i));
    }

    

}
