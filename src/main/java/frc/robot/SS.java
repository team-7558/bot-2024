package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.Setpoints;
import frc.robot.subsystems.shooter.Shooter.TargetMode;
import frc.robot.subsystems.shooter.ShotPresets;
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

    SPITTING,
    INTAKING,
    INTAKING_DEEP,
    INTAKING_CORAL,

    AMP_SCORING,

    AUTOPRECHAMBER,
    AUTOSHOOTING,
    AUTOCHAMBER,

    PRECHAMBER,
    CHAMBER,

    TRACKING,
    SHOOTER_SPIT,
    SHOOTING,
    SHOOTING_FROM_GROUND,
    SOURCE_FEEDING,
    SOURCE_SHOOTING,

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

  private boolean hasGamePiece, homedShooter, homedClimb, constrained;

  private SS() {
    lastState = State.DISABLED;
    currState = State.DISABLED;
    nextState = State.DISABLED;

    timer = new Timer();

    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    elevator = Elevator.getInstance();

    hasGamePiece = false;
    homedShooter = false;
    homedClimb = false;
    constrained = false;
  }

  public void queueState(State s) {
    if (currState != s) {
      System.out.println(currState + " -> " + s);
      nextState = s;
    }
  }

  private void stateInfoLog() {
    Logger.recordOutput("SS/lastState", lastState);
    Logger.recordOutput("SS/currState", lastState);
    Logger.recordOutput("SS/nextState", lastState);
    Logger.recordOutput("SS/stateTime", timer.get());
    Logger.recordOutput("SS/hasGamepiece", hasGamePiece);
    Logger.recordOutput("SS/Constrained", constrained);
  }

  public void periodic() {
    boolean first = currState != lastState;
    lastState = currState;
    if (currState != nextState) {
      timer.restart();
      currState = nextState;
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
          shooter.setCurrentState(shooter.HOLD);
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
          if (homedClimb && homedShooter) {
            queueState(State.IDLE);
          } else {
            intake.setCurrentState(intake.IDLE);
            shooter.setCurrentState(shooter.ZEROING);
            elevator.setCurrentState(elevator.HOMING);
          }
        }

        if (elevator.isState(elevator.IDLE)) {
          homedClimb = true;
        }

        if (shooter.isState(shooter.IDLE)) {
          homedShooter = true;
        }

        if (homedClimb && homedShooter) {
          queueState(State.IDLE);
        }
        break;
      case RESETTING_ELEVATOR:
        if (first) {
          homedClimb = false;
          elevator.setCurrentState(elevator.HOMING);
          intake.setCurrentState(intake.IDLE);
        }

        if (elevator.isState(elevator.IDLE)) {
          homedClimb = true;
        }

        if (homedClimb) {
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
      case SPITTING:
        if (first) {
          intake.setCurrentState(intake.SPITTING);
          hasGamePiece = false;
        }
        break;
      case INTAKING_DEEP:
      case INTAKING_CORAL:
      case INTAKING:
        if (first) {
          // elevator.setTargetHeight(Elevator.MIN_HEIGHT_M);
          // elevator.setCurrentState(elevator.TRAVELLING);
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
      case AMP_SCORING:
        if (first) {
          hasGamePiece = false;
          intake.setCurrentState(intake.AMP_SCORING);
          // elevator.setTargetHeight(Elevator.MIN_HEIGHT_M + 0.02);
          // elevator.setCurrentState(elevator.TRAVELLING);
        }

        break;

      case SOURCE_FEEDING:
        if (first) {
          elevator.setTargetHeight(elevator.MIN_HEIGHT_M + 0.115);
          elevator.setCurrentState(elevator.TRAVELLING);
        }
        if (intake.beamBroken()) {
          hasGamePiece = true;
        }

        if (hasGamePiece && !intake.beamBroken()) {
          // queueState(State.IDLE);
          intake.setCurrentState(intake.GHOSTING);
        } else if (!hasGamePiece) {
          intake.setCurrentState(intake.SOURCE_FEEDING);
        }

        if (hasGamePiece) {
          intake.setCurrentState(intake.FAST_SHOOTER);
        }

        if (shooter.beamBroken() || shooter.beamBrokenIn()) {
          hasGamePiece = false;
        }

        shooter.setCurrentState(shooter.SOURCE_FEEDING);

        break;

      case CLIMBING_UP:
        if (first) {
          elevator.setTargetHeight(Elevator.CLIMB_HEIGHT_M);
          shooter.queueSetpoints(new Setpoints(0, 0, 0, 0.05));
          shooter.setCurrentState(shooter.TRACKING);
          elevator.setCurrentState(elevator.TRAVELLING);
        }
        break;
      case CLIMBING_DOWN:
        if (first) {
          elevator.setTargetHeight(Elevator.MIN_HEIGHT_M);
          shooter.queueSetpoints(new Setpoints(0, 0, 0, 0.11));
          shooter.setCurrentState(shooter.TRACKING);
          elevator.setCurrentState(elevator.TRAVELLING);
        }
        break;
      case SHOOTER_SPIT:
        if (first) {

          intake.setCurrentState(intake.FEEDING);
          shooter.setCurrentState(shooter.SPITTING);
          hasGamePiece = false;
        }
        break;
      case AUTOPRECHAMBER:
        if (first) {
          if (!shooter.beamBroken()) {
            shooter.queueSetpoints(
                new Setpoints(Setpoints.DEFAULT, 0, 0, Shooter.PIVOT_MAX_FEED_POS_r));
            shooter.setCurrentState(shooter.TRACKING);
          } else {
            queueState(State.AUTOCHAMBER);
          }
        }

        if (shooter.isTurretAtSetpoint(0.03) && shooter.isPivotAtSetpoint(0.03)) {
          queueState(State.AUTOCHAMBER);
        }
        break;
      case AUTOCHAMBER:
        if (first) {
          shooter.setCurrentState(shooter.FAST_FEED);
          if (!shooter.beamBroken()) {
            intake.setCurrentState(intake.FAST_FEED);
          }
        }

        if (shooter.isState(shooter.IDLE)) {
          queueState(State.IDLE);
        }
        break;
      case PRECHAMBER:
        if (first) {
          if (!shooter.beamBroken()) {
            shooter.setCurrentState(shooter.TRACKING);
          } else {
            queueState(State.CHAMBER);
          }
        }

        if (shooter.isTurretAtSetpoint(0.01) && shooter.isPivotAtSetpoint(0.01)) {
          queueState(State.CHAMBER);
        }
        break;
      case CHAMBER:
        if (first) {
          shooter.setCurrentState(shooter.BEING_FED);
          if (!shooter.beamBroken() && !shooter.beamBrokenIn()) {
            intake.setCurrentState(intake.SHOOTER_SIDE);
          }
        }

        if (shooter.isState(shooter.IDLE)) {
          queueState(State.IDLE);
        }
        break;
      case TRACKING:
        if (first) {
          intake.setCurrentState(intake.IDLE);
          shooter.setCurrentState(shooter.TRACKING);
        }
        break;
      case SHOOTING:
        if (shooter.isAtSetpoints()) {
          hasGamePiece = false;
          // intake.setCurrentState(intake.SHOOTER_SIDE);
          shooter.setCurrentState(shooter.SHOOTING);
        }
        break;

      case AUTOSHOOTING:
        hasGamePiece = false;
        // intake.setCurrentState(intake.SHOOTER_SIDE);
        shooter.setCurrentState(shooter.SHOOTING);
        break;
      case SHOOTING_FROM_GROUND:
        if (first) {
          shooter.setCurrentState(shooter.SHOOTING);
          intake.setCurrentState(intake.SHOOTER_SIDE);
        }
        break;
      default:
        System.out.println(currState + " unimplemented state");
    }

    stateInfoLog();

    boolean flash = Math.sin(timer.get() * 20) > 0.0 ? true : false;

    if ((currState == State.INTAKING && hasGamePiece)
        || (currState == State.CHAMBER && intake.beamBroken())) {
      OI.DR.setRumble(RumbleType.kLeftRumble, 0.3);
      if (flash) {
        LED.getInstance().drawRow(0, 255, 255, 255);
        LED.getInstance().setBlinkin(0.93);
      }
    } else if (currState == State.TRACKING) {
      if (shooter.isAtSetpoints() && !constrained) {
        OI.DR.setRumble(RumbleType.kBothRumble, 0.6);
        if (flash) {
          LED.getInstance().drawRow(0, 0, 255, 0);
          LED.getInstance().setBlinkin(0.77);
        }
      } else {
        LED.getInstance().drawRow(0, 255, 0, 0);
      }
    } else {
      OI.DR.setRumble(RumbleType.kBothRumble, 0);
    }

    if (!homedShooter) {
      LED.getInstance().drawRow(5, 128, 0, 0);
      LED.getInstance().drawRow(6, 0, 128, 0);
      LED.getInstance().drawRow(7, 0, 0, 128);
    }

    if (!homedClimb) {
      LED.getInstance().drawRow(2, 128, 0, 0);
      LED.getInstance().drawRow(3, 0, 128, 0);
      LED.getInstance().drawRow(4, 0, 0, 128);
    }
  }

  public void action(State s) {
    queueState(State.TEST_2);
  }

  public void idle() {
    if (currState != State.IDLE && currState != State.BOOT) {
      if (currState == State.CLIMBING_UP) {
        queueState(State.CLIMBING_DOWN);
      } else if (currState == State.CLIMBING_DOWN) {
        // queueState(State.CLIMBING_DOWN);
      } else if (!elevator.atHeight(Elevator.MIN_HEIGHT_M, 0.01)) {
        queueState(State.RESETTING_ELEVATOR);
      } else {
        queueState(State.IDLE);
      }
    }
  }

  public void idleFromShooting() {
    if (currState == State.SHOOTING
        || currState == State.SHOOTING_FROM_GROUND
        || currState == State.PRECHAMBER
        || currState == State.CHAMBER) {
      // if (!elevator.atHeight(Elevator.MIN_HEIGHT_M, 0.01)) {
      //   queueState(State.RESETTING_ELEVATOR);
      // } else {
      //   queueState(State.IDLE);
      // }
    }
  }

  public void shooterSpit() {
    if (currState != State.BOOT) {
      queueState(State.SHOOTER_SPIT);
    }
  }

  public void spit() {
    if (currState != State.BOOT) {
      queueState(State.SPITTING);
    }
  }

  public void intake() {
    if (currState != State.BOOT) {
      queueState(State.INTAKING);
    }
  }

  public void amp() {
    if (currState != State.BOOT) {
      queueState(State.AMP_SCORING);
    }
  }

  public void chamber() {
    if (currState != State.BOOT && currState != State.CHAMBER && currState != State.PRECHAMBER) {
      if (shooter.beamBroken()) {
        queueState(State.IDLE);
      } else {
        queueSetpoints(new Setpoints(0, 0, 0, Shooter.PIVOT_MIN_FEED_POS_r));
        queueState(State.PRECHAMBER);
      }
    }
  }

  public void shoot() {
    if (currState != State.BOOT) {
      queueState(State.SHOOTING);
    }
  }

  public void autoShoot() {
    if (currState != State.BOOT) {
      queueState(State.AUTOSHOOTING);
    }
  }

  public void track() {
    if (currState != State.BOOT) {
      queueState(State.TRACKING);
    }
  }

  public void sourceFeed() {
    if (currState != State.BOOT) {
      queueState(State.SOURCE_FEEDING);
    }
  }

  public void sourceShoot() {
    if (currState != State.BOOT) {
      queueState(State.SOURCE_SHOOTING);
    }
  }

  public void trackFromPose() {
    if (currState != State.BOOT) {
      queueSetpointsLive();
      queueState(State.TRACKING);
    }
  }

  public void trackPreset(Setpoints s, boolean adjust) {
    if (currState != State.BOOT) {
      Setpoints sp = adjust ? shooter.adjustPreset(s) : s;
      if (shooter.beamBroken()) {
        Setpoints cs = shooter.constrainSetpoints(sp, false, false);
        shooter.queueSetpoints(cs);
        constrained = !cs.equals(sp);
        queueState(State.TRACKING);
      } else if (currState != State.CHAMBER && currState != State.PRECHAMBER) {
        Setpoints cs = shooter.constrainSetpoints(sp, true, false);
        shooter.queueSetpoints(cs);
        constrained = !cs.equals(sp);
        queueState(State.PRECHAMBER);
      } else if (currState == State.CHAMBER) {
        Setpoints cs = shooter.constrainSetpoints(sp, true, false);
        shooter.queueSetpoints(cs);
        constrained = !cs.equals(sp);
      }
    }
  }

  public void autoPreset(Setpoints s) {
    if (currState != State.BOOT) {
      if (shooter.beamBroken()) {
        trackPreset(shooter.constrainSetpoints(s, false, false), true);
      } else if (currState != State.AUTOCHAMBER && currState != State.AUTOPRECHAMBER) {
        queueState(State.AUTOPRECHAMBER);
      }
    }
  }

  public void autoPresetNoTurret(Setpoints s) {
    if (currState != State.BOOT) {
      if (shooter.beamBroken()) {
        // trackPreset(shooter.constrainSetpoints(s, false, false), false);
        shooter.queueSetpoints(s);
        queueState(State.TRACKING);
      } else if (currState != State.AUTOCHAMBER && currState != State.AUTOPRECHAMBER) {
        queueState(State.AUTOPRECHAMBER);
      }
    }
  }

  public void trackTrap() {
    if (currState != State.BOOT) {
      shooter.queueSetpoints(ShotPresets.TRAP_SHOT);
      constrained = false;
      queueState(State.TRACKING);
    }
  }

  public void climbUp() {
    if (currState != State.BOOT && currState != State.CLIMBING_UP) {
      queueState(State.CLIMBING_UP);
    }
  }

  public boolean hasGamePiece() {
    return hasGamePiece;
  }

  public void clearGamePiece() {
    hasGamePiece = false;
  }

  // Subsystem management

  public void queueSetpoints(Setpoints setpoints) {
    Setpoints cs = shooter.constrainSetpoints(setpoints, false, false);
    shooter.queueSetpoints(cs);
    constrained = !cs.equals(setpoints);
  }

  public void queueSetpointsLive() {
    shooter.setTargetMode(TargetMode.SPEAKER);
    Setpoints ps = shooter.shooterPipeline();
    Setpoints cs = shooter.constrainSetpoints(ps, false, false);
    shooter.queueSetpoints(cs);
    constrained = !cs.equals(ps);
  }

  public void resetHomingFlags() {
    homedClimb = false;
    homedShooter = false;
  }

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

  public boolean isDisabled() {
    return currState == State.DISABLED;
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
