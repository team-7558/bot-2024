// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.G;
import frc.robot.OI;
import frc.robot.SS;
import frc.robot.SS.State;
import frc.robot.SS2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter.Setpoints;
import frc.robot.subsystems.shooter.ShotPresets;
import frc.robot.subsystems.shooter.TurretCamIO.Pipeline;
import frc.robot.util.Util;

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
      } else if (OI.DR.getYButton() || (ss.isInShooter() && (OI.XK.get(2, 2) || OI.XK.get(4, 3)))) {
        drive.setAutolockSetpoint(G.isRedAlliance() ? 0.5 : 0);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getBButton()) {
        drive.setAutolockSetpoint(
            G.isRedAlliance() ? Units.radiansToRotations(1.054) : Units.radiansToRotations(-2.088));
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getXButton()) {
        drive.setAutolockSetpoint(
            G.isRedAlliance() ? Units.radiansToRotations(-1.028) : Units.radiansToRotations(2.072));
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getBackButton()) {
        drive.setCurrentState(drive.INTAKING);
      } else if (OI.XK.get(9, 7)) {
        drive.setCurrentState(drive.TRAPPING);
      } else {
        // strafe and turn if not other state
        drive.setCurrentState(drive.STRAFE_N_TURN);
      }
    }

    if (OI.DR.getPOV() == 270 || OI.XK.get(9, 3)) {
      ss.clearGamePiece();
    }

    if (!ss.isDisabled()) {

      if (OI.DR.getPOV() == 90 && OI.DR.getStartButton()) {
        ss.resetHomingFlags();
        ss.queueState(State.BOOT);
      } else if (OI.XK.get(5, 5)) {
        ss.amp();
      } else if (OI.XK.get(5, 7)) {
        ss.climbUp();
      } else if (OI.XK.get(7, 3)) {
        ss.spit();
      } else if (OI.XK.get(8, 3)) {
        ss.shooterSpit();
      } else if (OI.DR.getLeftTriggerAxis() > 0.05) {
        ss.shoot();
      } else if (OI.XK.get(4, 5) || OI.DR.getBackButton()) {
        ss.chamber();
      } else if (OI.XK.get(8, 2)) {
        ss.trackTrap();
        ss.setLastPreset("TRAP");
      } else if (OI.XK.get(3, 3)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_FENDER : ShotPresets.BLUE_FENDER,
            Pipeline.NEAR,
            true,
            false);
        ss.setLastPreset("FENDER");
      } else if (OI.XK.get(4, 2)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_AMP_BOX : ShotPresets.BLUE_AMP_BOX,
            Pipeline.NEAR,
            true,
            true);
        ss.setLastPreset("AMP BOX");
      } else if (OI.XK.get(1, 3)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_SIDE_POST : ShotPresets.BLUE_SIDE_POST,
            Pipeline.FAR,
            true,
            true);
        ss.setLastPreset("SIDE POST");
      } else if (OI.XK.get(0, 2)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_FRONT_POST : ShotPresets.BLUE_FRONT_POST,
            Pipeline.NEAR,
            true,
            true);
        ss.setLastPreset("FRONT POST");
      } else if (OI.XK.get(1, 2)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_FRONT_COURT : ShotPresets.BLUE_FRONT_COURT,
            Pipeline.NEAR,
            true,
            true);
        ss.setLastPreset("FRONT COURT");
      } else if (OI.XK.get(2, 3)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.OP_SHOT : ShotPresets.BLUE_OP_SHOT,
            Pipeline.NEAR,
            true,
            true);
        ss.setLastPreset("OP");
      } else if (OI.XK.get(2, 2)) {
        Setpoints s = G.isRedAlliance() ? ShotPresets.RED_CLEAR_WALL : ShotPresets.BLUE_CLEAR_WALL;
        double speedMag = Math.abs(drive.getChassisSpeeds().vxMetersPerSecond);
        s.flywheel_rps = Util.remap(0, 5.0, speedMag, 24, ShotPresets.RED_CLEAR_WALL.flywheel_rps);
        ss.trackPreset(s, Pipeline.FAR, false, false);
        ss.setLastPreset("CLEAR WALL");
      } else if (OI.XK.get(4, 3)) {
        Setpoints s = G.isRedAlliance() ? ShotPresets.RED_CLEAR_MID : ShotPresets.BLUE_CLEAR_MID;
        double speedMag = Math.abs(drive.getChassisSpeeds().vxMetersPerSecond);
        s.flywheel_rps = Util.remap(0, 5.0, speedMag, 24, ShotPresets.RED_CLEAR_MID.flywheel_rps);
        ss.trackPreset(s, Pipeline.FAR, false, false);
        ss.setLastPreset("CLEAR MID");
      } else if (OI.XK.get(0, 3)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.SNIPE : ShotPresets.BLUE_SNIPE,
            Pipeline.FAR,
            true,
            true);
        ss.setLastPreset("SNIPE");
      } else if (OI.XK.get(3, 2)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_AMP_SNIPE : ShotPresets.BLUE_AMP_SNIPE,
            Pipeline.FAR,
            true,
            true);
        ss.setLastPreset("AMP SNIPE");
      } else if (OI.XK.get(2, 1)) {
        ss.trackPreset(
            G.isRedAlliance() ? ShotPresets.RED_STEAL_SHOT : ShotPresets.BLUE_STEAL_SHOT,
            Pipeline.FAR,
            true,
            true);
        ss.setLastPreset("STEAL");
      } else if (OI.DR.getLeftBumper() || OI.XK.get(6, 5)) {
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
