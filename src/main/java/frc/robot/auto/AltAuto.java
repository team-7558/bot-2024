package frc.robot.auto;

import frc.robot.subsystems.drive.Drive;

public abstract class AltAuto {
    
    protected final Trajstack trajstack;

    protected final Drive drive;

    public AltAuto(){
        drive = Drive.getInstance();
        trajstack = new Trajstack();
    }

    protected abstract void onInit();
    protected abstract void onExecute();


    public final void init(){

    }

    public final void execute(){


    }



    

}
