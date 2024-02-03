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
  private TalonFX left, right, feeder;
  private DutyCycleOut out;
  private DutyCycleOut feederOut;

  /** Creates a new DriveTeleop. */
  public Temp() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();

    left = new TalonFX(20);
    right = new TalonFX(21);
    feeder = new TalonFX(25);

    out = new DutyCycleOut(0);
    feederOut = new DutyCycleOut(0);

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

    if (OI.DR.getAButton()) {
      out = new DutyCycleOut(0.10);
    } else if (OI.DR.getBButton()) {
      out = new DutyCycleOut(-0.1);
    } else {
      out = new DutyCycleOut(0);
    }

    if (OI.DR.getRightTriggerAxis() > 0) {
      feederOut = new DutyCycleOut(0.5);
    } else if (OI.DR.getLeftTriggerAxis() > 0) {
      feederOut = new DutyCycleOut(0.4);
    } else {
      feederOut = new DutyCycleOut(0);
    }

    left.setControl(out);
    right.setControl(out);

    feeder.setControl(feederOut);
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
