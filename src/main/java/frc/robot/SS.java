package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

// The Super Structure
public class SS {

  public enum State {
    DISABLED,
    IDLE,
    MANUAL,
    TEST_2,
    TEST_3,
    INTAKING,
    INTAKING_DEEP,
    INTAKING_CORAL,
    AMP_SCORING_UP,
    AMP_SCORING_DOWN,
    SHOOTING,
    CLIMBING,
    ENDGAME
  }

  private static SS instance;

  public static SS getInstance() {
    if (instance == null) {
      instance = new SS();
    }

    return instance;
  }

  private Timer timer;

  private Intake intake;
  private Shooter shooter;
  private Elevator elevator;

  private State lastState;
  private State currState;
  private State nextState;

  private boolean hasGamePiece;

  private SS() {
    lastState = State.DISABLED;
    currState = State.DISABLED;
    nextState = State.DISABLED;

    timer = new Timer();

    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    elevator = Elevator.getInstance();

    hasGamePiece = false;
  }

  public void queueState(State s) {
    nextState = s;
  }

  private void stateInfoLog() {
    Logger.recordOutput("SS/lastState", lastState);
    Logger.recordOutput("SS/currState", lastState);
    Logger.recordOutput("SS/nextState", lastState);
    Logger.recordOutput("SS/stateTime", timer.get());
    Logger.recordOutput("SS/hasGamepiece", hasGamePiece);
  }

  public void periodic() {
    boolean first = currState != lastState;
    boolean last = nextState != currState;
    lastState = currState;

    // Control Switch
    switch (currState) {
      case DISABLED:
        if (first) {
          intake.setCurrentState(intake.DISABLED);
          shooter.setCurrentState(shooter.DISABLED);
          elevator.setCurrentState(elevator.DISABLED);
        }
        break;
      case IDLE:
        if (first) {
          intake.setCurrentState(intake.IDLE);
          shooter.setCurrentState(shooter.IDLE);
          elevator.setCurrentState(elevator.IDLE);
        }

        break;
      case MANUAL:
        if (first) {
          intake.setCurrentState(intake.IDLE);
          shooter.setCurrentState(shooter.IDLE);
          elevator.setCurrentState(elevator.IDLE);
        }

        break;
      case TEST_2:
        if (after(3)) {
          queueState(State.TEST_3);
        }
        break;
      case TEST_3:
      case INTAKING_DEEP:
      case INTAKING_CORAL:
      case INTAKING:
        if (first) {
          elevator.setTargetHeight(Elevator.INTAKE_HEIGHT_M);
          elevator.setCurrentState(elevator.TRAVELLING);
        }

        if (intake.beamBroken()) {
          hasGamePiece = true;
        }

        if (hasGamePiece && !intake.beamBroken()) {
          queueState(State.IDLE);
          intake.setCurrentState(intake.IDLE);
        } else if (!hasGamePiece) {
          intake.setCurrentState(intake.INTAKING);
        }

        break;
      case AMP_SCORING_UP:
        if (first) {
          elevator.setTargetHeight(Elevator.AMP_HEIGHT_M);
          elevator.setCurrentState(elevator.TRAVELLING);

          intake.setCurrentState(intake.AMP_SIDE_1);
        }
        if (last) {
          elevator.setTargetHeight(Elevator.RESET_HEIGHT_M);
          elevator.setCurrentState(elevator.TRAVELLING);

          intake.setCurrentState(intake.AMP_SIDE_2);
        }
        if (elevator.isState(elevator.HOLDING)) {
          queueState(State.AMP_SCORING_DOWN);
        }

        break;
      case AMP_SCORING_DOWN:
        if (elevator.isState(elevator.HOLDING)) {
          elevator.setCurrentState(elevator.HOMING);
          hasGamePiece = false;
        }

        if (elevator.isState(elevator.IDLE)) {
          queueState(State.IDLE);
        }

        break;
      case SHOOTING:
        hasGamePiece = false;
      case CLIMBING:
      default:
        System.out.println(currState + " unimplemented state");
    }

    stateInfoLog();

    // Figure out if its better to do this before or after the switch statement
    if (last) {
      currState = nextState;
      timer.restart();
    }
  }

  // TODO: replace example based on what you wanna do, ex. shoot(double x, double y)
  public void action(State s) {
    System.out.println("Switching to state " + s);
    queueState(s);
  }

  public void idle() {
    if (currState != State.IDLE) {
      queueState(State.IDLE);
    }
  }

  public void intake() {
    if (currState == State.IDLE) {
      queueState(State.INTAKING);
    }
  }

  public void amp() {
    if (currState == State.IDLE) {
      queueState(State.AMP_SCORING_UP);
    }
  }

  public void shoot() {
    if (currState == State.IDLE) {
      queueState(State.INTAKING);
    }
  }

  public void climb() {
    if (currState == State.IDLE) {
      queueState(State.INTAKING);
    }
  }

  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  // Subsystem management

  public void disableIntake() {
    intake.setCurrentState(intake.DISABLED);
  }

  public void disableElevator() {
    elevator.setCurrentState(elevator.DISABLED);
  }

  public void disableShooter() {
    shooter.setCurrentState(shooter.DISABLED);
  }

  public boolean intakeIsDisabled() {
    return intake.isState(intake.DISABLED);
  }

  public boolean elevatorIsDisabled() {
    return elevator.isState(elevator.DISABLED);
  }

  public boolean shooterIsDisabled() {
    return shooter.isState(shooter.DISABLED);
  }

  // Timer Functions
  public boolean after(double seconds) {
    return timer.hasElapsed(seconds);
  }

  public boolean before(double seconds) {
    return !timer.hasElapsed(seconds);
  }

  public boolean between(double earlier, double later) {
    double t = timer.get();
    return earlier < t && t < later;
  }

  public double alpha(double earlier, double later) {
    double t = timer.get();
    if (t <= earlier) return 0.0;
    if (t >= later) return 1.0;
    return Util.unlerp(earlier, later, t);
  }
}
