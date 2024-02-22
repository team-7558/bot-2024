// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.SS2d;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StateMachineSubsystemBase {
  public static final double MIN_HEIGHT_M = Units.inchesToMeters(25.5); // 0.6096;
  public static final double MAX_HEIGHT_M = Units.inchesToMeters(36.25); // 0.8763;
  public static final double STROKE_M = MAX_HEIGHT_M - MIN_HEIGHT_M;
  public static final double MAX_VEL_MPS = 1.0;
  public static final double CURRENT_THRESHOLD_A = 10;

  private static Elevator instance;

  public static Elevator getInstance() {

    if (instance == null) {
      System.out.println("Elevator initialized");
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance = new Elevator(new ElevatorIOReal());
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new Elevator(new ElevatorIOIdeal());
          break;

        default:
          // Replayed robot, disable IO implementations
          instance = new Elevator(new ElevatorIO() {});
          break;
      }
    }
    return instance;
  }

  public final State DISABLED, IDLE, HOLDING, CLIMBING, HOMING, RESETTING, MANUAL;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged(); // hacky fix

  private double targetHeight_m = MIN_HEIGHT_M;
  private double manualOutput = 0;

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

    IDLE =
        new State("IDLE") { // Should only be IDLE at bottom
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
            io.climb(targetHeight_m);
          }
        };
    HOMING =
        new State("HOMING") {
          @Override
          public void init() {
            targetHeight_m = MIN_HEIGHT_M;
          }

          @Override
          public void periodic() {
            if (isHomeStalling()) {
              System.out.println("AWWWWW");
              setCurrentState(RESETTING);
            } else {
              // io.setVel(-0.05);
              io.setVoltage(-0.5);
            }
          }
        };

    RESETTING =
        new State("RESETTING") {
          @Override
          public void init() {
            targetHeight_m = MIN_HEIGHT_M;
            stop();
          }

          @Override
          public void periodic() {
            if (after(0.5)) {
              resetPosAsMin();
              setCurrentState(IDLE);
            }
          }
        };

    MANUAL =
        new State("MANUAL") {
          @Override
          public void init() {
            stop();
          }

          @Override
          public void periodic() {
            io.setVoltage(manualOutput);
          }

          @Override
          public void exit() {
            manualOutput = 0;
          }
        };

    setCurrentState(DISABLED);
  }

  public void setTargetHeight(double height_m) {
    this.targetHeight_m = MathUtil.clamp(height_m, MIN_HEIGHT_M, MAX_HEIGHT_M);
  }

  public double getTargetHeight() {
    return this.targetHeight_m;
  }

  public double getHeight() {
    return inputs.posMeters;
  }

  public void setManualOutput(double manualOutput) {
    this.manualOutput = manualOutput;
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  @Override
  public void outputPeriodic() {
    SS2d.M.setElevatorHeight(inputs.posMeters);
    SS2d.S.setElevatorHeight(targetHeight_m);
    Logger.recordOutput("Elevator/TargetHeight_m", targetHeight_m);
  }

  public void stop() {
    io.stop();
  }

  public boolean isHomeStalling() {
    return (inputs.currents[0] + inputs.currents[1]) >= CURRENT_THRESHOLD_A;
  }

  public void resetPosAsMin() {
    io.resetPos(MIN_HEIGHT_M);
  }
}
