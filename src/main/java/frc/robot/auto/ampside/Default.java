package frc.robot.auto.ampside;

import frc.robot.SS;
import frc.robot.auto.AltAuto;

public class Default extends AltAuto{

    public Default() {
        super("DefaultAmpside");
        trajstack.appendChain().append("Default Ampside", false);
        trajstack.setActiveIdx(0);
        
        trajstack.generate();
    }

    @Override
    public void onInit() {
    }

    @Override
    public void onExecute() {

        if(after(1.35) && before(1.8)) {
            SS.getInstance().intake();
        }

        if()

    }
    
}
