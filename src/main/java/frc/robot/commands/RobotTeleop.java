// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.TargetMode;

public class RobotTeleop extends Command {

  private final Drive drive;
  private final Shooter shooter;

  /** Creates a new DriveTeleop. */
  public RobotTeleop() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();
    shooter = Shooter.getInstance();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.STRAFE_N_TURN);
    shooter.setCurrentState(shooter.LOCKONT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!shooter.isState(shooter.DISABLED)) {
      if (OI.DR.getAButton()) { // make actually binded
        shooter.setTargetMode(TargetMode.TRAP);
      } else {
        shooter.setTargetMode(TargetMode.SPEAKER);
      }
    }

    if (!drive.isState(drive.DISABLED)) {
      // slow mode
      // x stance while shooting
      if (OI.DR.getLeftTriggerAxis() > 0) {
        drive.setCurrentState(drive.SHOOTING);
      } else
      // autolocking
      if (OI.DR.getXButton()) {
        drive.setAutolockSetpoint(-61.19);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getAButton()) {
        drive.setAutolockSetpoint(0);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getBButton()) {
        drive.setAutolockSetpoint(59.04);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getYButton()) {
        drive.setAutolockSetpoint(90);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else {
        // strafe and turn if not other state
        drive.setCurrentState(drive.STRAFE_N_TURN);
      }

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
