// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.SS;
import frc.robot.SS.State;
import frc.robot.SS2d;
import frc.robot.subsystems.drive.Drive;

public class RobotTeleop extends Command {

  private final Drive drive;
  private final SS ss;

  /** Creates a new DriveTeleop. */
  public RobotTeleop() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();
    ss = SS.getInstance();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.STRAFE_N_TURN);
    ss.queueState(State.IDLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!drive.isState(drive.DISABLED)) {
      // slow mode
      // x stance while shooting
      if (OI.DR.getPOV() == 180) {
        drive.hardSetPose(new Pose2d());
        drive.zeroGyro();
      } else if (OI.DR.getXButton()) {

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


    if (!ss.intakeIsDisabled()) {

      if (OI.DR.getAButton()) {
        ss.intake();
      } else if (OI.DR.getAButtonReleased()) {
        ss.idle();
      }
    }

    if (!ss.shooterIsDisabled()) {
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
    ss.queueState(State.DISABLED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
