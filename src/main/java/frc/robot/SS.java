package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReason;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.util.Util;

//The Super Structure
public class SS {

    public enum State{
        DISABLED,
        IDLE,
        TEST_2,
        TEST_3,
        INTAKING,        
        INTAKING_DEEP,
        INTAKING_CORAL,
        SHOOTING,
        CLIMBING,
        ENDGAME

    }

    private static SS instance;

    public static SS getInstance() {
        if (instance == null){
            instance = new SS();
        }

        return instance;
    }


    private Timer timer;

    private State lastState;
    private State currState;    
    private State nextState;

    private SS(){
        lastState = State.DISABLED;        
        currState = State.DISABLED;
        nextState = State.DISABLED;

        timer = new Timer();
    }

    public void queueState(State s){
        nextState = s;
    }

    private void sateInfoLog(){
        Logger.recordOutput("SS/lastState", lastState);
        Logger.recordOutput("SS/currState", lastState);
        Logger.recordOutput("SS/nextState", lastState);
        Logger.recordOutput("SS/stateTime", timer.get());

    }

    public void periodic(){
        boolean first = currState != lastState;        
        boolean last = nextState != currState;
        lastState = currState;

        //Control Switch
        switch(currState){
            case DISABLED:
                break;
            case IDLE:
                break;
            case TEST_2:
                if(after(3)){
                    queueState(State.TEST_3);
                }
                break;
            case TEST_3:            
            case INTAKING:        
            case INTAKING_DEEP:
            case INTAKING_CORAL:
            case SHOOTING:
            case CLIMBING:
            default:
                System.out.println(currState + " unimplemented state");
        }

        // Figure out if its better to do this before or after the switch statement
        if(last){ 
            currState = nextState;
            timer.restart();
        }
    }

    //TODO: replace example based on what you wanna do, ex. shoot(double x, double y)
    public void action(State s){
        System.out.println("Switching to state " + s);
        queueState(s);
    }



    //Timer Functions
    public boolean after(double seconds) {
        return timer.hasElapsed(seconds);
    }

    public boolean before(double seconds) {
        return !timer.hasElapsed(seconds);
    }

    public boolean between(double earlier, double later) {
        double t = timer.get();
        return earlier < t && t < later;
    }

    public double alpha(double earlier, double later) {
        double t = timer.get();
        if (t <= earlier) return 0.0;
        if (t >= later) return 1.0;
        return Util.unlerp(earlier, later, t);
    } 
}
