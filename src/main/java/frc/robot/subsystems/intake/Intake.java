// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachineSubsystemBase {

  // Convention: towards amp is positive, towards shooter is negative

  private double intakeSpeed = 0;
  private double directionSpeed = 0;

  private static Intake instance;

  public static Intake getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          // Real robot, instantiate hardware IO implementations
          instance = new Intake(new IntakeIOTalonFX());
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

  public final State DISABLED, IDLE, INTAKING, SHOOTER_SIDE, AMP_SIDE_1, AMP_SIDE_2, SPITTING;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private Intake(IntakeIO io) {
    super("Intake");
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
        };

    IDLE =
        new State("IDLE") {

          @Override
          public void init() {
            stop();
          }
        };
    INTAKING =
        new State("INTAKING") {
          @Override
          public void init() {
            intakeSpeed = 0.1;
            directionSpeed = 0;
          }
        };

    AMP_SIDE_1 =
        new State("AMP_SIDE_1") {
          @Override
          public void init() {
            directionSpeed = 0;
            intakeSpeed = 0.1;
          }
        };
    AMP_SIDE_2 =
        new State("AMP_SIDE_2") {
          @Override
          public void init() {
            directionSpeed = 0.1;
            intakeSpeed = 0.1;
          }
        };
    SHOOTER_SIDE =
        new State("SHOOTER_SIDE") {
          @Override
          public void init() {
            directionSpeed = -0.1;
            intakeSpeed = 0.1;
          }
        };
    SPITTING =
        new State("SPITTING") {
          @Override
          public void init() {
            intakeSpeed = -0.1;
            directionSpeed = 0.0;
          }
        };

    setCurrentState(DISABLED);
  }

  public void stop() {
    intakeSpeed = 0;
    directionSpeed = 0;
    io.stop();
  }

  public boolean beamBroken() {
    return inputs.beamBreakActivated; // TODO: determine if this needs a debouncer
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void outputPeriodic() {
    io.setDirectionSpeed(directionSpeed);
    io.setIntakeSpeed(intakeSpeed);
    Logger.recordOutput("Intake/TargetBottomSpeed", intakeSpeed);
    Logger.recordOutput("Intake/TargetTopSpeed", directionSpeed);
    Logger.recordOutput("Intake/DirectorWheelSpeedRPM", getDirectionVelocityRPM());
    Logger.recordOutput("Intake/IntakeWheelSpeedRPM", getIntakeVelocityRPM());
  }

  public double getIntakeVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.intakeVelocityRadPerSec);
  }

  public double getDirectionVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.directionVelocityRadPerSec);
  }

  public void runCharacterizationVolts(double volts) {
    io.setIntakeVoltage(volts);
  }

  public double getCharacterizationVelocity() {
    return inputs.intakeVelocityRadPerSec;
  }
}
