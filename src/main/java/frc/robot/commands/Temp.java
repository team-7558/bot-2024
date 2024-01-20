// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;

public class Temp extends Command {

  private final Drive drive;
  private TalonFX left, right;
  private DutyCycleOut lout, rout;

  /** Creates a new DriveTeleop. */
  public Temp() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();

    left = new TalonFX(31);
    right = new TalonFX(32);

    lout = new DutyCycleOut(0);
    rout = new DutyCycleOut(0);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.DISABLED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!drive.isState(drive.DISABLED)) {
      // slow mode
      // x stance while shooting
      if (OI.DR.getLeftTriggerAxis() > 0) {
        drive.setCurrentState(drive.SHOOTING);
      } else
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
        drive.setCurrentState(drive.STRAFE_N_TURN);
      // }

      // if (OI.XK.get(0, 0)) {
      //   drive.setModuleModes(Mode.VOLTAGE);
      // } else if (OI.XK.get(0, 1)) {
      //   drive.setModuleModes(Mode.SETPOINT);
      // }
    }

    if (OI.DR.getAButton()) {
      lout.withOutput(0.25);
      rout.withOutput(0.25);
    } else if (OI.DR.getBButton()) {
      lout.withOutput(0.5);
      rout.withOutput(0.5);
    } else if (OI.DR.getYButton()) {
      lout.withOutput(0.75);
      rout.withOutput(0.75);
    } else if (OI.DR.getXButton()) {
      lout.withOutput(1.0);
      rout.withOutput(1.0);
    } else {
      lout.withOutput(0);
      rout.withOutput(0);
    }

    left.setControl(lout);
    right.setControl(rout);
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
