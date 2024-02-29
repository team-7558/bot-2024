package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.Util;

public class ElevatorBringup extends Command {
  private final Drive drive;
  private final Elevator elevator;

  public ElevatorBringup() {
    drive = Drive.getInstance();
    elevator = Elevator.getInstance();
    addRequirements(drive, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.DISABLED);
    elevator.setCurrentState(elevator.HOMING);
  }

  boolean unlocked = true;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!elevator.isState(elevator.DISABLED)) {
      if (OI.DR.getLeftBumper()) {
        elevator.setCurrentState(elevator.HOMING);
      }

      boolean up = OI.DR.getPOV() == 0, down = OI.DR.getPOV() == 180;
      double manual = 1.0 * (up ? 1 : down ? -1 : 0);
      if (up || down) {
        elevator.setManualOutput(manual);
        elevator.setCurrentState(elevator.MANUAL);
      } else if (OI.DR.getXButton()) {
        elevator.setTargetHeight(Util.lerp(Elevator.MIN_HEIGHT_M, Elevator.MAX_HEIGHT_M, 0.2));
        elevator.setCurrentState(elevator.TRAVELLING);
      } else if (OI.DR.getBButton()) {
        elevator.setTargetHeight(Util.lerp(Elevator.MIN_HEIGHT_M, Elevator.MAX_HEIGHT_M, 0.8));
        elevator.setCurrentState(elevator.TRAVELLING);
      } else if (!elevator.isState(elevator.IDLE)
          && !elevator.isState(elevator.HOMING)
          && !elevator.isState(elevator.RESETTING)) {
        if (OI.DR.getYButtonPressed()) unlocked = !unlocked;
        if (unlocked) {
          elevator.setTargetHeight(elevator.getHeight());
        }
        elevator.setCurrentState(elevator.HOLDING);
      }
    }

    if (OI.DR.getRightBumper()) {
      elevator.resetPosAsMin();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
