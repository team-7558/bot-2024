// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;

/** Add your docs here. */
public class TurretCamIOReal implements TurretCamIO {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tid = table.getEntry("tid");
  NetworkTableEntry sc = table.getEntry("snapshot");

  LimelightResults res;

  @Override
  public void updateInputs(TurretCamIOInputs inputs) {
    res = LimelightHelpers.getLatestResults("");

    LimelightTarget_Fiducial[] fids = res.targetingResults.targets_Fiducials;

    boolean v = res.targetingResults.valid;

    inputs.connected = true;
    inputs.tv = v;
    inputs.latency =
        res.targetingResults.latency_pipeline
            + res.targetingResults.latency_capture
            + res.targetingResults.latency_jsonParse;
    inputs.ids = fids.length;
    inputs.tx = v ? fids[0].tx : 0;
    inputs.ty = v ? fids[0].ty : 0;
    inputs.ta = v ? fids[0].ta : 0;
    inputs.tid = v ? fids[0].fiducialID : 0;
  }

  Pipeline pipeline = Pipeline.FAR;

  @Override
  public void setPipeline(Pipeline pipeline) {
    if (pipeline == Pipeline.FAR) {
      LimelightHelpers.setPipelineIndex("", 0);
    } else if (pipeline == Pipeline.NEAR) {
      LimelightHelpers.setPipelineIndex("", 1);
    } else if (pipeline == Pipeline.TRAP) {
      LimelightHelpers.setPipelineIndex("", 2);
    }
  }

  @Override
  public void snapshot() {
    LimelightHelpers.takeSnapshot("", "Shot_" + Preferences.getLong("shotUUID", -1));
  }
}
