// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class ObjectDetectorIOReal implements ObjectDetectorIO {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  @Override
  public void updateInputs(ObjectDetectorIOInputs inputs) {
    inputs.connected = true;
    inputs.tv = tv.getDouble(0) != 0 ? true : false;
    inputs.tx = tx.getDouble(0);
    inputs.ty = ty.getDouble(0);
    inputs.ta = ta.getDouble(0);
  }

  @Override
  public void setPipeline(int id) {}
}
