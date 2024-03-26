// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.SS2d;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StateMachineSubsystemBase {
  public static final double MIN_HEIGHT_M = Units.inchesToMeters(25.5); // 0.6096;
  public static final double MAX_HEIGHT_M = Units.inchesToMeters(36.25); // 0.8763;
  public static final double STROKE_M = MAX_HEIGHT_M - MIN_HEIGHT_M;

  public static final double INTAKE_HEIGHT_M = MIN_HEIGHT_M;
  public static final double RESET_HEIGHT_M = MIN_HEIGHT_M + 0.005;
  public static final double MIN_FEED_HEIGHT_M = MIN_HEIGHT_M;
  public static final double MAX_FEED_HEIGHT_M = MIN_HEIGHT_M;
  public static final double AMP_HEIGHT_M =
      MAX_HEIGHT_M - Units.inchesToMeters(1); // 2 inches above the bottom
  public static final double CLIMB_HEIGHT_M = MAX_HEIGHT_M - 0.01;
  public static final double MAX_VEL_MPS = 1.0;
  public static final double CURRENT_THRESHOLD_A = 50;

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
          instance = new Elevator(new ElevatorIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          instance = new Elevator(new ElevatorIO() {});
          break;
      }
    }
    return instance;
  }

  public final State DISABLED, IDLE, HOLDING, TRAVELLING, HOMING, RESETTING, MANUAL;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

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
            if (atTargetHeight()) {
              io.holdPos(targetHeight_m);
            } else {
              setCurrentState(TRAVELLING);
            }
          }
        };
    TRAVELLING =
        new State("TRAVELLING") {
          @Override
          public void periodic() {
            if (!atTargetHeight()) {
              io.travelToPos(targetHeight_m);
            } else {
              setCurrentState(HOLDING);
            }
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
              setCurrentState(RESETTING);
            } else {
              // io.setVel(-0.05);
              io.setVoltage(-0.75);
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
            stop();
            manualOutput = 0;
          }
        };

    setCurrentState(DISABLED);
  }

  public boolean atHeight(double height_m, double tol) {
    return Util.inRange(height_m - inputs.posMeters, tol);
  }

  public boolean atTargetHeight() {
    return atTargetHeight(0.02);
  }

  public boolean atTargetHeight(double tol) {
    return atHeight(targetHeight_m, tol);
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

  public void toggleBrake() {
    io.toggleBrake();
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
    return inputs.hallEffect;
  }

  public void resetPosAsMin() {
    io.resetPos(MIN_HEIGHT_M);
  }
}
