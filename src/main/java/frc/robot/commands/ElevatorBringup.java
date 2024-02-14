package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

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
    elevator.setCurrentState(elevator.IDLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!elevator.isState(elevator.DISABLED)) {
      if (OI.DR.getLeftBumper()) {
        elevator.setCurrentState(elevator.HOMING);
      } else {
        elevator.setCurrentState(elevator.IDLE);
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
