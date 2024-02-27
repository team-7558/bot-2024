package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.Setpoints;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

// The Super Structure
public class SS {

  public enum State {
    DISABLED,
    IDLE,
    BOOT,
    RESETTING_ELEVATOR,
    MANUAL,

    TEST_2,
    TEST_3,

    INTAKING,
    INTAKING_DEEP,
    INTAKING_CORAL,

    AMP_SCORING_CHAMBER,
    AMP_SCORING_UP,
    AMP_SCORING_DOWN,

    PRECHAMBER,
    CHAMBER,

    TRACKING,
    SHOOTING,

    CLIMBING_UP,
    CLIMBING_DOWN,

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
    lastState = currState;

    if (nextState != currState) {
      currState = nextState;
      timer.restart();
    }

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
          shooter.setCurrentState(shooter.IDLE);
          elevator.setCurrentState(elevator.IDLE);
        }
        if (hasGamePiece) {
          intake.setCurrentState(intake.GHOSTING);
        } else {
          intake.setCurrentState(intake.IDLE);
        }
        break;
      case BOOT:
        if (first) {
          intake.setCurrentState(intake.IDLE);
          shooter.setCurrentState(shooter.ZEROING);
          elevator.setCurrentState(elevator.HOMING);
        }

        if (elevator.isState(elevator.IDLE) && shooter.isState(shooter.IDLE)) {
          queueState(State.IDLE);
        }
        break;
      case RESETTING_ELEVATOR:
        if (first) {
          elevator.setCurrentState(elevator.HOMING);
          intake.setCurrentState(intake.IDLE);
        }

        if (elevator.isState(elevator.IDLE)) {
          queueState(State.IDLE);
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
        if (first) {}
        elevator.setTargetHeight(0.5 * (Elevator.MAX_HEIGHT_M + Elevator.MIN_HEIGHT_M));
        elevator.setCurrentState(elevator.TRAVELLING);
        break;
      case TEST_3:
        break;
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
          // queueState(State.IDLE);
          intake.setCurrentState(intake.GHOSTING);
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

        break;
      case AMP_SCORING_DOWN:
        if (first) {
          elevator.setTargetHeight(Elevator.RESET_HEIGHT_M);
          elevator.setCurrentState(elevator.TRAVELLING);

          intake.setCurrentState(intake.AMP_SIDE_2);
          hasGamePiece = false;
        }

        if (elevator.isState(elevator.HOLDING) && after(.02)) {
          queueState(State.RESETTING_ELEVATOR);
        }
        break;
      case CLIMBING_UP:
        if (first) {
          elevator.setTargetHeight(Elevator.CLIMB_HEIGHT_M);
          elevator.setCurrentState(elevator.TRAVELLING);
        }
        break;
      case CLIMBING_DOWN:
        if (first) {
          elevator.setTargetHeight(Elevator.MIN_HEIGHT_M);
          elevator.setCurrentState(elevator.TRAVELLING);
        }
        break;
      case PRECHAMBER:
        if (first) {
          elevator.setTargetHeight(Elevator.MIN_FEED_HEIGHT_M);
          elevator.setCurrentState(elevator.TRAVELLING);
          shooter.queueSetpoints(new Setpoints(0, 0, 0, 0.0));
          shooter.setCurrentState(shooter.TRACKING);
        }

        if (shooter.isAtSetpoints() && elevator.isState(elevator.HOLDING)) {
          queueState(State.CHAMBER);
        }
        break;
      case CHAMBER:
        if (first) {
          shooter.setCurrentState(shooter.BEING_FED);
          intake.setCurrentState(intake.SHOOTER_SIDE);
        }

        if (shooter.isState(shooter.IDLE)) {
          queueState(State.IDLE);
        }
        break;
      case TRACKING:
        if (first) {
          shooter.setCurrentState(shooter.TRACKING);
        }
        break;
      case SHOOTING:
        if (first) {
          shooter.setCurrentState(shooter.TRACKING);
        }

        if (shooter.isAtSetpoints() && after(0.2)) {
          hasGamePiece = false;
          shooter.setCurrentState(shooter.SHOOTING);
        }
        break;
      default:
        System.out.println(currState + " unimplemented state");
    }

    stateInfoLog();
  }

  // TODO: replace example based on what you wanna do, ex. shoot(double x, double y)
  public void action(State s) {
    System.out.println("Switching to state " + s);
    queueState(State.TEST_2);
  }

  public void idle() {
    if (currState != State.IDLE) {
      if (!elevator.atHeight(Elevator.MIN_HEIGHT_M, 0.01)) {
        queueState(State.RESETTING_ELEVATOR);
      } else {
        queueState(State.IDLE);
      }
    }
  }

  public void intake() {
    if (currState == State.IDLE) {
      queueState(State.INTAKING);
    }
  }

  public void ampUp() {
    if (currState == State.IDLE) {
      queueState(State.AMP_SCORING_UP);
    }
  }

  public void ampDown() {
    if (currState == State.AMP_SCORING_UP && elevator.isState(elevator.HOLDING)) {
      queueState(State.AMP_SCORING_DOWN);
    }
  }

  public void chamber() {
    if (currState == State.IDLE) {
      queueState(State.PRECHAMBER);
    }
  }

  public void shoot() {
    if (currState == State.IDLE) {
      queueState(State.SHOOTING);
    }
  }

  public void shootPreset1() {
    if (currState == State.IDLE) {
      shooter.queueSetpoints(new Setpoints(30, 0, 0, 0.18));
      queueState(State.SHOOTING);
    }
  }

  public void shootPreset2() {
    if (currState == State.IDLE) {
      shooter.queueSetpoints(new Setpoints(40, 0, 0, 0.102));
      queueState(State.SHOOTING);
    }
  }

  public void climbUp() {
    if (currState == State.IDLE || currState == State.CLIMBING_DOWN) {
      queueState(State.CLIMBING_UP);
    }
  }

  public void climbDown() {
    if (currState == State.CLIMBING_UP) {
      queueState(State.CLIMBING_DOWN);
    }
  }

  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  public void clearGamePiece() {
    hasGamePiece = false;
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
