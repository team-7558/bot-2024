// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.G;
import frc.robot.OI;
import frc.robot.SS;
import frc.robot.SS.State;
import frc.robot.SS2d;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShotPresets;

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
    ss.queueState(State.BOOT);
    ss.disableShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean usingLeds = false;

    if (!drive.isState(drive.DISABLED)) {
      // slow mode
      // x stance while shooting
      if (OI.DR.getAButton()) {
        drive.setAutolockSetpoint(0.25);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getBButton()) {
        drive.setAutolockSetpoint(G.isRedAlliance() ? 0.15 : -0.35);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getXButton()) {
        drive.setAutolockSetpoint(G.isRedAlliance() ? -0.15 : -0.35);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getYButton()) {
        drive.setAutolockSetpoint(G.isRedAlliance() ? 0.5 : 0);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getBackButton()) {
        drive.setCurrentState(drive.INTAKING);
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
      } else if (OI.XK.get(7, 7)) {
        ss.climbUp();
      } else if (OI.XK.get(7, 0)) {
        ss.chamber();
      } else if (OI.XK.get(7, 2)) {
        ss.spit();
      } else if (OI.XK.get(7, 4)) {
        ss.shooterSpit();
      } else if (OI.DR.getLeftTriggerAxis() > 0.05) {
        ss.shoot();
      } else if (OI.XK.get(9, 7)) {
        ss.trackPreset(ShotPresets.TRAP_SHOT, false);
      } else if (OI.XK.get(0, 0)) {
        ss.trackPreset(G.isRedAlliance() ? ShotPresets.RED_FENDER : ShotPresets.BLUE_FENDER, true);
      } else if (OI.XK.get(0, 1)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_FRONT_POST : ShotPresets.BLUE_FRONT_POST, true);
      } else if (OI.XK.get(0, 2)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_SIDE_POST : ShotPresets.BLUE_SIDE_POST, true);
      } else if (OI.XK.get(0, 3)) {
        System.err.println("HUUUUH");
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_AMP_BOX : ShotPresets.BLUE_AMP_BOX, true);
      } else if (OI.XK.get(0, 4)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_FRONT_COURT : ShotPresets.BLUE_FRONT_COURT, true);
      } else if (OI.XK.get(0, 5)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_WING_POST : ShotPresets.BLUE_WING_POST, true);
      } else if (OI.XK.get(0, 6)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_WING_WALL : ShotPresets.BLUE_WING_WALL, true);
      } else if (OI.XK.get(9, 0)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_CLEAR_WALL : ShotPresets.BLUE_CLEAR_WALL, true);
      } else if (OI.XK.get(9, 1)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_CLEAR_MID : ShotPresets.BLUE_CLEAR_MID, true);
      } else if (OI.XK.get(9, 2)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_CLEAR_CLOSE : ShotPresets.BLUE_CLEAR_CLOSE, true);
      } else if (OI.DR.getLeftBumper() || OI.DR.getBackButton()) {
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
