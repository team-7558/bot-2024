package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {

    NetworkTable limelight;

    public VisionIOLimelight(String limelightName) {
        limelight = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.xOffset = limelight.getEntry("tx").getDouble(0);
        inputs.yOffset = limelight.getEntry("ty").getDouble(0);
        inputs.pipelineID = limelight.getEntry("pl").getDouble(-1);
        inputs.tagID = (int) limelight.getEntry("tid").getDouble(-1);

        double[] botpose_double = limelight.getEntry("botpose").getDoubleArray(new double[7]);
        inputs.pose =
            new Pose2d(botpose_double[0], botpose_double[1], Rotation2d.fromDegrees(botpose_double[5]));
        inputs.latency = botpose_double[6];
        inputs.timestamp = System.currentTimeMillis() - botpose_double[6];
    }

    @Override
    public void setPipeline(int pipelineID) {
        limelight.getEntry("pl").setDouble(pipelineID);
    }

}
