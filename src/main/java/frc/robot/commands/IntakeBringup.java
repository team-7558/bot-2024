// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

public class Temp extends Command {

  private final Drive drive;
  private final Intake intake;

  boolean hasGamePiece = false;

  /** Creates a new DriveTeleop. */
  public Temp() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();
    intake = Intake.getInstance();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.DISABLED);
    intake.setCurrentState(intake.IDLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!intake.isState(intake.DISABLED)) {

      if (intake.beamBroken()) {
        hasGamePiece = true;
      } else if (OI.DR.getRightTriggerAxis() > 0) {
        hasGamePiece = false;
      }

      if (OI.DR.getAButton()) {
        if (hasGamePiece && !intake.beamBroken()) {
          intake.setCurrentState(intake.IDLE);
        } else if (!hasGamePiece) {
          intake.setCurrentState(intake.INTAKING);
        }
      } else if (OI.DR.getBButton()) {
        intake.setCurrentState(intake.AMP_SIDE_2);
      } else if (OI.DR.getXButton()) {
        intake.setCurrentState(intake.SHOOTER_SIDE);
      } else {
        intake.setCurrentState(intake.IDLE);
      }
    }

    if (!drive.isState(drive.DISABLED)) {
      // slow mode
      // x stance while shooting
      // autolocking
      // if (OI.DR.getXButton()) {
      //   drive.setAutolockSetpoint(-61.19);
      //   drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      // } else if (OI.DR.getAButton()) {
      //   drive.setAutolockSetpoint(0);
      //   drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      // } else if (OI.DR.getBButton()) {
      //   drive.setAutolockSetpoint(59.04);
      //   drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      // } else if (OI.DR.getYButton()) {
      //   drive.setAutolockSetpoint(90);
      //   drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      // } else {
      // strafe and turn if not other state
      // drive.setCurrentState(drive.STRAFE_N_TURN);
      // }

      // if (OI.XK.get(0, 0)) {
      //   drive.setModuleModes(Mode.VOLTAGE);
      // } else if (OI.XK.get(0, 1)) {
      //   drive.setModuleModes(Mode.SETPOINT);
      // }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setCurrentState(drive.DISABLED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
