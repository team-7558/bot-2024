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

public class RobotTeleop extends Command {

  private final Drive drive;
  private final Intake intake;

  /** Creates a new DriveTeleop. */
  public RobotTeleop() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();
    intake = Intake.getInstance();
    addRequirements(drive, intake);
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

    if (!drive.isState(drive.DISABLED)) {
      // slow mode
      // x stance while shooting
      if (OI.DR.getPOV() == 180) {
        drive.setPose(new Pose2d());
      } else
      // autolocking
      if (OI.DR.getXButton()) {
        drive.setAutolockSetpoint(-61.19); // TODO: make source
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getAButton()) { // TODO: make speaker
        drive.setAutolockSetpoint(0);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getBButton()) { // TOOD: make amp
        drive.setAutolockSetpoint(59.04);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getYButton()) { // TODO: make trap
        drive.setAutolockSetpoint(90);
        drive.setCurrentState(drive.STRAFE_AUTOLOCK);
      } else if (OI.DR.getLeftTriggerAxis() > 0) { // TODO: auto drive to set location

      } else if (OI.XK.get(0, 7)) { // zero gyro bottom left of xkeys
        drive.zeroGyro();
      } else if (OI.XK.get(2, 5)) { // INTAKE SPEAKER

      } else if (OI.XK.get(0, 5)) { // INTAKE AMP

      } else if (OI.XK.get(2, 1)) { // SHOOT AT TARGET

      } else if (OI.XK.get(5, 0)) { // TRAP SCORING

      } else if (OI.XK.get(0, 0)) { // HANG UP

      } else if (OI.XK.get(0, 1)) { // HANG DOWn

      }

      // according to marco and lucca's wants

      else {
        // strafe and turn if not other state
        drive.setCurrentState(drive.STRAFE_N_TURN);
      }

      // if (OI.XK.get(0, 0)) {
      //   drive.setModuleModes(Mode.VOLTAGE);
      // } else if (OI.XK.get(0, 1)) {
      //   drive.setModuleModes(Mode.SETPOINT);
      // }
    }

    if (OI.DR.getAButton()) SS2d.S.setElevatorHeight(SS2d.GROUND_TO_MAX_HEIGHT);
    if (OI.DR.getBButton()) SS2d.S.setElevatorHeight(SS2d.GROUND_TO_MIN_HEIGHT);

    if (OI.DR.getLeftTriggerAxis() > 0) SS2d.S.setIntakeMotors(1, 1);
    else if (OI.DR.getLeftBumper()) SS2d.S.setIntakeMotors(1, -1);
    else SS2d.S.setIntakeMotors(0, 0);

    if (OI.DR.getRightBumper()) SS2d.S.setShooterTilt(45);
    else SS2d.S.setShooterTilt(0);

    if (OI.DR.getPOV() != -1) SS2d.S.setTurretAngle(OI.DR.getPOV() * 0.5 - 90);

    SS2d.S.setTurretBaseAngle(drive.getRotation());
    SS2d.S.setDistance(drive.getPose().getX());

    if (!intake.isState(intake.DISABLED)) {
      if (OI.DR.getRightBumper()) {
        intake.setCurrentState(intake.AMP_SIDE_2);
      } else if (OI.DR.getLeftBumper()) {
        intake.setCurrentState(intake.SHOOTER_SIDE);
      } else {
        intake.setCurrentState(intake.IDLE);
      }
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
