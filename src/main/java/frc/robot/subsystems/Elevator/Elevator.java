// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;

public class Elevator extends StateMachineSubsystemBase {
  public static final double ELEVATOR_MIN_HEIGHT_M = 0.5;
  public static final double ELEVATOR_MAX_HEIGHT_M = 1.0;
  public static final double ELEVATOR_STROKE_M = ELEVATOR_MAX_HEIGHT_M - ELEVATOR_MIN_HEIGHT_M;


  private static Elevator instance;

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator(new ElevatorIOIdeal());
    }
    return instance;
  }

  public final State DISABLED, IDLING, HOLDING, CLIMBING, HOMING;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double targetHeight_m = ELEVATOR_MIN_HEIGHT_M;

  private Elevator(ElevatorIO io) {
    super("Elevator");
  
    this.io = io;

    DISABLED =
    new State("DISABLED") {
      
      @Override
      public void init() {
        stop();
      }
    };
    
    IDLING =
    new State("IDLING") {
      @Override
      public void init() {
        stop();
      }
    };
    
    HOLDING =
    new State("HOLDING") {
      @Override
      public void periodic() {
        io.setPos(targetHeight_m);
      }
    };
    CLIMBING =
    new State("CLIMBING") {
      @Override
      public void periodic() {
        io.setPos(targetHeight_m);
      }
    };
    HOMING =
    new State("HOMING") {
      @Override
      public void init() {
        targetHeight_m = ELEVATOR_MIN_HEIGHT_M;
      }
      @Override
      public void periodic() {
        io.setVel(-0.1);
      }
    };
    
    setCurrentState(DISABLED);
  }

  public void setTargetHeight(double height_m){
    this.targetHeight_m = MathUtil.clamp(height_m, ELEVATOR_MIN_HEIGHT_M, ELEVATOR_MAX_HEIGHT_M);
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }
  
  @Override
  public void outputPeriodic() {
    Logger.recordOutput("Elevator/TargetHeight_m", targetHeight_m);
  }
  
  public void stop() {
    io.stop();
  }
}
