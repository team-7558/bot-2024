// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.SS2d;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachineSubsystemBase {

  // Convention: towards amp is positive, towards shooter is negative

  public double intakeSpeed = 0;
  public double directionSpeed = 0;

  private static Intake instance;

  public static Intake getInstance() {
    if (instance == null) {
      System.out.println("Intake initialized");
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
            directionSpeed = 0;
          }

          @Override
          public void periodic() {
            if (beamBroken()) stop();
            else intakeSpeed = 0; // -0.05;
          }
        };
    INTAKING =
        new State("INTAKING") {
          @Override
          public void periodic() {
            intakeSpeed = beamBroken() ? 0.2 : 0.40;
            directionSpeed = 0;
          }
        };

    AMP_SIDE_1 =
        new State("AMP_SIDE_1") {
          @Override
          public void init() {
            directionSpeed = 0;
            intakeSpeed = 0.25;
          }
        };
    AMP_SIDE_2 =
        new State("AMP_SIDE_2") {
          @Override
          public void init() {
            directionSpeed = 0.5;
            intakeSpeed = 0.5;
          }
        };
    SHOOTER_SIDE =
        new State("SHOOTER_SIDE") {
          @Override
          public void init() {
            directionSpeed = -0.5;
            intakeSpeed = 0.5;
          }
        };
    SPITTING =
        new State("SPITTING") {
          @Override
          public void init() {
            intakeSpeed = -0.25;
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
    SS2d.M.setIntakeMotors(inputs.intakeVelocityRadPerSec, inputs.directionVelocityRadPerSec);
    SS2d.S.setIntakeMotors(intakeSpeed, directionSpeed);
    Logger.recordOutput("Intake/TargetBottomSpeed", intakeSpeed);
    Logger.recordOutput("Intake/TargetTopSpeed", directionSpeed);
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
