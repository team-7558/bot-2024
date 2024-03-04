// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.G;
import frc.robot.OI;
import frc.robot.SS;
import frc.robot.SS.State;
import frc.robot.SS2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class RobotTeleop extends Command {

  private final Drive drive;
  private final SS ss;
  private final Vision v;

  /** Creates a new DriveTeleop. */
  public RobotTeleop() {

    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();
    ss = SS.getInstance();
    v = Vision.getInstance();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.STRAFE_N_TURN);
    ss.queueState(State.BOOT);
    ss.disableShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!drive.isState(drive.DISABLED)) {
      // slow mode
      // x stance while shooting
      if (OI.DR.getPOV() == 180) {
        drive.hardSetPose(
            new Pose2d(
                drive.getPose().getTranslation(),
                Rotation2d.fromRotations(G.isRedAlliance() ? 0.5 : 0)));
      } else if (OI.DR.getAButton()) {
        drive.setAutolockSetpoint(0.25);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getBButton()) {
        drive.setAutolockSetpoint(G.isRedAlliance() ? 0.35 : -0.35);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getXButton()) {
        drive.setAutolockSetpoint(G.isRedAlliance() ? 0.15 : -0.15);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getYButton()) {
        drive.setAutolockSetpoint(G.isRedAlliance() ? 0.5 : 0);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else {
        // strafe and turn if not other state
        drive.setCurrentState(drive.STRAFE_N_TURN);
      }
    }

    if (OI.DR.getPOV() == 270 || OI.XK.get(0, 6)) {
      ss.clearGamePiece();
    }

    if (!ss.isDisabled()) {

      if (OI.DR.getPOV() == 90 || OI.XK.get(0, 7)) {
        ss.resetHomingFlags();
        ss.queueState(State.BOOT);
      } else if (OI.DR.getRightBumper()) {
        ss.amp();
      } else if (OI.XK.get(5, 0)) {
        ss.climbUp();
      } else if (OI.XK.get(7, 0)) {
        ss.chamber();
      } else if (OI.XK.get(6, 0)) {
        ss.spit();
      } else if (OI.DR.getLeftTriggerAxis() > 0.05) {
        ss.shoot();
      } else if (OI.XK.get(7, 1)) {
        ss.trackAmp();
      } else if (OI.XK.get(8, 0)) {
        ss.trackFender();
      } else if (OI.XK.get(8, 1)) {
        ss.trackFrontPost();
      } else if (OI.XK.get(8, 2)) {
        ss.trackSidePost();
      } else if (OI.XK.get(8, 3)) {
        ss.trackAmpBox();
      } else if (OI.XK.get(8, 4)) {
        ss.trackFrontCourt();
      } else if (OI.XK.get(8, 5)) {
        ss.trackWingPost();
      } else if (OI.XK.get(8, 6)) {
        ss.trackWingWall();
      } else if (OI.XK.get(8, 7)) {
        ss.trackWallClear();
      } else if (OI.DR.getLeftBumper()) {
        ss.intake();
      } else {
        ss.idle();
      }
    }

    SS2d.S.setTurretBaseAngle(drive.getRotation());
    SS2d.M.setTurretBaseAngle(drive.getRotation());

    SS2d.S.setDistance(2.5);
    SS2d.M.setDistance(5);
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
