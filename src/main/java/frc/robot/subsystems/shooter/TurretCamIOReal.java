// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.G;

/** Add your docs here. */
public class TurretCamIOReal implements TurretCamIO {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tid = table.getEntry("tid");
  NetworkTableEntry pid = table.getEntry("priorityid");
  NetworkTableEntry pipeline = table.getEntry("pipeline");
  NetworkTableEntry sc = table.getEntry("snapshot");

  @Override
  public void updateInputs(TurretCamIOInputs inputs) {
    inputs.connected = true;
    inputs.tv = tv.getDouble(0) != 0 ? true : false;
    inputs.tx = tx.getDouble(0);
    inputs.ty = ty.getDouble(0);
    inputs.ta = ta.getDouble(0);
    inputs.tid = tid.getDouble(0);
    sc.setDouble(0);
  }

  @Override
  public void setPipeline(Pipeline pipeline) {
    if (pipeline == Pipeline.NEAR) {
      pid.setDouble(G.isRedAlliance() ? 4 : 7);
      this.pipeline.setDouble(0);
    } else if (pipeline == Pipeline.FAR) {
      pid.setDouble(G.isRedAlliance() ? 4 : 7);
      this.pipeline.setDouble(1);
    } else {
      pid.setDouble(G.isRedAlliance() ? 5 : 6);
      this.pipeline.setDouble(2);
    }
  }

  @Override
  public void snapshot() {
    sc.setDouble(1);
  }
}
