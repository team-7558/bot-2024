// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.SS2d;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StateMachineSubsystemBase {
  public static final double MIN_HEIGHT_M = 0.6096;
  public static final double MAX_HEIGHT_M = 0.8763;
  public static final double STROKE_M = MAX_HEIGHT_M - MIN_HEIGHT_M;
  public static final double MAX_SPEED_V = 12.0;

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

  public final State DISABLED, IDLE, HOLDING, CLIMBING, HOMING;

  private final ElevatorIO io;
  private final ElevatorIOInputsAL inputs = new ElevatorIOInputsAL(); // hacky fix

  private double targetHeight_m = MIN_HEIGHT_M;

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
        new State("IDLE") {
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
            io.setVel(-0.1);
          }
        };

    setCurrentState(DISABLED);
  }

  public void setTargetHeight(double height_m) {
    this.targetHeight_m = MathUtil.clamp(height_m, MIN_HEIGHT_M, MAX_HEIGHT_M);
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  @Override
  public void outputPeriodic() {
    SS2d.M.setElevatorHeight(inputs.pos_m);
    SS2d.S.setElevatorHeight(targetHeight_m);
    Logger.recordOutput("Elevator/TargetHeight_m", targetHeight_m);
  }

  public void stop() {
    io.stop();
  }
}
