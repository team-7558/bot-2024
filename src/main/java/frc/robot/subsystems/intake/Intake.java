// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder.IndexingType;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;


public class Intake extends StateMachineSubsystemBase {

  private static Intake instance;

  public static Intake getInstance() {
    if(instance == null) {
      switch (Constants.currentMode) {
        case REAL: 
          // Real robot, instantiate hardware IO implementations
          // instance = new Intake(new IntakeIOTalonFX()); // TODO: make IndexerIOTalonFX
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new Intake(new IntakeIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          instance = new Intake(new IntakeIO() {});
          break;
      } 
    }
    return instance;
  }

  public final State DISABLED, IDLE, INDEXING, CORRECTING;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Intake(IntakeIO io) {
    super("Indexer");
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0,0);
        io.configurePID(0, 0, 0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0, 0);
        io.configurePID(0, 0, 0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0, 0);
        break;
    }

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

    IDLE = 
      new State("IDLE") {

        @Override
        public void init() {
          stop();
        }

        @Override
        public void periodic() {
          if(inputs.ballSensor) {
            setCurrentState(INDEXING);
          }
        }

        @Override
        public void exit() {}
      };
    INDEXING = 
      new State("INDEXING") {
        @Override
        public void init() {
          io.configurePID(0.01,0,0);
          io.configurePID2(0.01,0,0);
        }

        @Override
        public void periodic() {
          io.setVelocity(0.1, 1);
          io.setVelocity2(0.1, 1);
          
          //TODO: Add the after, before alpha thingyt

          if(!inputs.ballSensor) {
            setCurrentState(IDLE);
          }
        }

        @Override
        public void exit() {}
      };
    CORRECTING =
      new State("CORRECTING"){
         @Override
        public void init() {
          io.configurePID(-0.01, 0, 0);
          io.configurePID2(-0.01, 0, 0);
        }

        @Override
        public void periodic() {
          io.setVelocity(0.1, 1);
          io.setVelocity2(0.1, 1);
        }

        @Override
        public void exit() {}
      };
      setCurrentState(DISABLED);
  }

  public void stop() {
    io.stop();
    io.stop2();
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IndexerWheel", inputs);
  }

  @Override
  public void outputPeriodic() {
    Logger.recordOutput("IndexerWheelSpeedRPM", getVelocityRPM());
  }

  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    Logger.recordOutput("IndexerWheelSetpointRPM", velocityRPM);
  }

  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public void runCharacterizationVolts(double volts) {
    io.setVoltage(volts);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

}