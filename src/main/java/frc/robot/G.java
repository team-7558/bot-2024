package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

//Globals
public class G {
    
    private static boolean allianceFound = false;
    private static boolean isRed = true;

    public static boolean isRedAlliance(){
        if(!allianceFound && DriverStation.getAlliance().isPresent()){
            isRed = DriverStation.getAlliance().get() == Alliance.Red;
            allianceFound = true;
        }
        return isRed;
    }

}
