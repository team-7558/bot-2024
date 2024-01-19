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

  public final State DISABLED, IDLE, INTAKING, SHOOTER_SIDE, AMP_SIDE, SPITTING;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Intake(IntakeIO io) {
    super("Indexer");
    this.io = io;

    // switch (Constants.currentMode) {
    //   case REAL:
    //   case REPLAY:
    //     break;
    //   case SIM:
    //     break;
    //   default:
    //     break;
    // }

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
          if(inputs.beamBreakActivatedBottom) {
            setCurrentState(INTAKING);
          }
        }

        @Override
        public void exit() {}
      };
    INTAKING = 
      new State("INTAKING") {
        boolean isPassedSensor;
        @Override
        public void init() {
          io.setIntakeSpeed(1);
          io.setElevatorSpeed(1);
          io.setDirectionSpeed(0);
          isPassedSensor = false;
        }

        @Override
        public void periodic() {
          if(inputs.beamBreakActivatedTop) {
            isPassedSensor = true;
          }

          if(!inputs.beamBreakActivatedTop && isPassedSensor) {
            io.stop();
          }
        }

        @Override
        public void exit() {}
      };
    AMP_SIDE =
      new State("AMP_SIDE"){
         @Override
        public void init() {
          io.setDirectionSpeed(-1);
        }

        @Override
        public void periodic() {
        }
        @Override
        public void exit() {}
      };
    SHOOTER_SIDE =
      new State("SHOOTER_SIDE"){
         @Override
        public void init() {
          io.setDirectionSpeed(1);
        }

        @Override
        public void periodic() {
        }
        @Override
        public void exit() {}
      };
    SPITTING =
      new State("SPITTING"){
         @Override
        public void init() {
          io.setIntakeSpeed(-1);
        }

        @Override
        public void periodic() {
        }
        @Override
        public void exit() {}
      };
  }

  public void stop() {
    io.stop();
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
    io.setIntakeVelocity(velocityRadPerSec);
    Logger.recordOutput("IntakeWheelSetpointRPM", velocityRPM);
  }

  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.intakeVelocityRadPerSec);
  }

  public void runCharacterizationVolts(double volts) {
    io.setIntakeVoltage(volts);
  }

  public double getCharacterizationVelocity() {
    return inputs.intakeVelocityRadPerSec;
  }

}