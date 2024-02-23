// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class ShooterBringup extends Command {

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;

  private Timer t;

  boolean hasGamePiece = false;

  /** Creates a new DriveTeleop. */
  public ShooterBringup() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();

    t = new Timer();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.DISABLED);
    intake.setCurrentState(intake.IDLE);
    shooter.setCurrentState(shooter.IDLE);

    t.reset();
    t.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!shooter.isState(shooter.DISABLED)) {
      double s = 0.025 * Math.sin(t.get() * 0.5);
      double c = 0.025 * Math.cos(t.get() * 1);

      if (OI.DR.getAButton()) {
        shooter.queueSetpoints(new Setpoints(30, 0, 0.11));
        shooter.setCurrentState(shooter.TRACKING);
      } else if (OI.DR.getBButton()) {
        intake.setCurrentState(intake.SHOOTER_SIDE);
        shooter.queueSetpoints(new Setpoints(45, 40, -0.15, 0.063));
        shooter.setCurrentState(shooter.TRACKING);
      } else {
        intake.setCurrentState(intake.IDLE);
        shooter.setCurrentState(shooter.IDLE);
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
