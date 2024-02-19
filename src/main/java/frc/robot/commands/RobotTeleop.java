// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.SS2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotTeleop extends Command {

  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  boolean hasGamePiece = false;

  /** Creates a new DriveTeleop. */
  public RobotTeleop() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.STRAFE_N_TURN);
    intake.setCurrentState(intake.IDLE);
    shooter.setCurrentState(shooter.IDLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!drive.isState(drive.DISABLED)) {
      // slow mode
      // x stance while shooting
      if (OI.DR.getPOV() == 180) {
        drive.setPose(new Pose2d());
        drive.zeroGyro();
      } else if (OI.DR.getXButton()) {

      } else {
        // strafe and turn if not other state
        drive.setCurrentState(drive.STRAFE_N_TURN);
      }
    }

    if (!intake.isState(intake.DISABLED)) {

      if (intake.beamBroken()) {
        hasGamePiece = true;
      } else if (OI.DR.getRightBumper()) {
        hasGamePiece = false;
      }

      if (OI.DR.getAButton()) {
        if (hasGamePiece && !intake.beamBroken()) {
          intake.setCurrentState(intake.IDLE);
        } else if (!hasGamePiece) {
          intake.setCurrentState(intake.INTAKING);
        }
      } else if (OI.DR.getBButton()) {
        intake.setCurrentState(intake.SHOOTER_SIDE);
      } else {
        intake.setCurrentState(intake.IDLE);
      }
    }

    if (!shooter.isState(shooter.DISABLED)) {
      // if (OI.DR.getLeftTriggerAxis() > 0) {
      //   shooter.setCurrentState(shooter.MANUAL);
      // } else {
      //   shooter.setCurrentState(shooter.IDLE);
      // }
    }

    SS2d.S.setTurretBaseAngle(drive.getRotation());
    SS2d.M.setTurretBaseAngle(drive.getRotation());

    SS2d.S.setDistance(2.5);
    SS2d.M.setDistance(5);

    // if (!intake.isState(intake.DISABLED)) {
    //   if (OI.DR.getRightBumper()) {
    //     intake.setCurrentState(intake.AMP_SIDE_2);
    //   } else if (OI.DR.getLeftBumper()) {
    //     intake.setCurrentState(intake.SHOOTER_SIDE);
    //   } else {
    //     intake.setCurrentState(intake.IDLE);
    //   }
    // }
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
