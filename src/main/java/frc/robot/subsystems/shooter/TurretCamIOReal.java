// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.LimelightTarget_Fiducial;

/** Add your docs here. */
public class TurretCamIOReal implements TurretCamIO {

  LimelightResults res;

  TurretCamIOInputs lastInputs = new TurretCamIOInputs();

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

    if (v) {
      inputs.tx = fids[0].tx;
      inputs.ty = fids[0].ty;
      inputs.ta = fids[0].ta;
      inputs.tid = fids[0].fiducialID;
    } else {
      inputs.tx = lastInputs.tx;
      inputs.ty = lastInputs.ty;
      inputs.ta = lastInputs.ta;
      inputs.tid = lastInputs.tid;
    }

    lastInputs.tx = inputs.tx;
    lastInputs.ty = inputs.ty;
    lastInputs.ta = inputs.ta;
    lastInputs.tid = inputs.tid;
  }

  Pipeline pipeline = Pipeline.FAR;

  @Override
  public void setPipeline(Pipeline pipeline) {
    if (pipeline == Pipeline.NEAR) {
      LimelightHelpers.setPipelineIndex("", 0);
    } else if (pipeline == Pipeline.FAR) {
      LimelightHelpers.setPipelineIndex("", 1);
    } else if (pipeline == Pipeline.TRAP) {
      LimelightHelpers.setPipelineIndex("", 2);
    }
  }

  @Override
  public void snapshot() {
    LimelightHelpers.takeSnapshot("", "Shot_" + Preferences.getLong("shotUUID", -1) + "_");
  }

  @Override
  public void setLEDs(LEDStatus on) {
    if (on == LEDStatus.HI) {
      LimelightHelpers.setLEDMode_ForceOn("");
    } else if (on == LEDStatus.LOW) {
      LimelightHelpers.setLEDMode_PipelineControl("");
    } else {
      LimelightHelpers.setLEDMode_ForceOff("");
    }
  }
}
