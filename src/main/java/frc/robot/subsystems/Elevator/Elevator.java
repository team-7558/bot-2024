// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.subsystems.StateMachineSubsystemBase;
public class Elevator extends StateMachineSubsystemBase {
    private static Elevator instance;
    public static Elevator getInstance() {
        if(instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
    private Elevator() {
        super("Elevator");
    }
    @Override
    public void inputPeriodic() {
        //Will be called once every scheduled run
    }

    @Override
    public void outputPeriodic() {
        //Will be called once every scheduled run
    }

    public final State DISABLED, IDLING, TRAVELING, THERE, HOMING; {
        
      DISABLED =
            new State("DISABLED") {
            
              @Override
            public void init() {
                stop();
            }
            
            @Override
            public void periodic() {}
            @Override
            public void exit() {}
            };
        
        IDLING =
            new State("IDLING") {
              @Override
              public void periodic() {}
            };
        
        TRAVELING =
            new State("TRAVELING") {
              @Override
              public void periodic() {}
            };
        THERE =
            new State("THERE") {
              @Override
              public void periodic() {}
            };
        HOMING =
            new State("HOMING") {
              @Override
              public void periodic() {}
            };

        setCurrentState(DISABLED);
        }
}

