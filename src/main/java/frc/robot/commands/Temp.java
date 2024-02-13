// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;

public class Temp extends Command {

  DutyCycleOut out;
  DutyCycleOut feederOut;
  TalonFX top = new TalonFX(9);
  TalonFX bottom = new TalonFX(10);
  TalonFX feeder = new TalonFX(8);

  private final Drive drive;

  /** Creates a new DriveTeleop. */
  public Temp() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();
    bottom.setControl(new Follower(top.getDeviceID(), false));
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
    out = new DutyCycleOut(0);
    feederOut = new DutyCycleOut(0);
    if (OI.DR.getAButton()) {
      out = new DutyCycleOut(-0.5);
      feederOut = new DutyCycleOut(0.2);
    }

    top.setControl(out.withEnableFOC(true));
    feeder.setControl(feederOut.withEnableFOC(true));
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
